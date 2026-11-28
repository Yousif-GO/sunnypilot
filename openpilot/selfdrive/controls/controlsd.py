#!/usr/bin/env python3
import math
from numbers import Number

from openpilot.cereal import log
from opendbc.car.structs import car
import openpilot.cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_curvature import LatControlCurvature
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls(ControlsExt):
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    # Initialize sunnypilot controlsd extension and base model state
    ControlsExt.__init__(self, self.CP, self.params)

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.CP_SP)

    self.sm = messaging.SubMaster(['lateralDelay', 'vehicleParameters', 'lateralTorqueParameters', 'modelV2', 'selfdriveState',
                                   'extrinsicsCalibration', 'deviceMotion', 'longitudinalPlan', 'lateralManeuverPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'navInstruction'] + self.sm_services_ext,
                                  poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'] + self.pm_services_ext)

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0
    self._prev_steer_angle_deg = 0.0
    self._nav_turn_nudged = False

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP, self.CP_SP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      self.LaC = LatControlCurvature(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CP_SP, self.CI, DT_CTRL)

    self.LaC = ControlsExt.initialize_lateral_control(self, self.LaC, self.CI, DT_CTRL)

  def update(self):
    self.sm.update(15)
    if self.sm.updated["extrinsicsCalibration"]:
      self.pose_calibrator.feed_extrinsics_calibration(self.sm['extrinsicsCalibration'])
    if self.sm.updated["deviceMotion"]:
      device_motion = Pose.from_device_motion(self.sm['deviceMotion'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_motion)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['vehicleParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['lateralTorqueParameters']
      if self.sm.all_checks(['lateralTorqueParameters']) and torque_params.useParams:
        self.LaC.update_torque_parameters(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

        self.LaC.extension.update_limits()

      self.LaC.extension.update_model_v2(self.sm['modelV2'])

      self.LaC.extension.update_lateral_lag(self.lat_delay)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill

    # Get which state to use for active lateral control
    _lat_active = self.get_lat_active(self.sm)

    # custom-c4: blinkers normally pause lateral steering (letting the driver wait safely for oncoming traffic),
    # EXCEPT when confirming an explicit navigation turn (left or right) with BOTH matching blinker AND a
    # deliberate steering hand-nudge in the turn direction. Nudging opposite instantly aborts the turn.
    nav_turn_eligible = False
    nav_left = False
    nav_right = False
    if self.sm.alive['navInstruction'] and self.sm.valid['navInstruction']:
      nav = self.sm['navInstruction']
      dist = nav.maneuverDistance
      mtype = (nav.maneuverType or "").lower()
      mmod = (nav.maneuverModifier or "").lower()
      trigger_dist = min(18.0, max(8.0, CS.vEgo * 2.5))
      is_explicit_turn = mtype in ("turn", "end of road") and "slight" not in mmod
      if -5.0 <= dist <= trigger_dist and is_explicit_turn and CS.vEgo < 10.0:
        nav_left = "left" in mmod and CS.leftBlinker and not CS.rightBlinker
        nav_right = "right" in mmod and CS.rightBlinker and not CS.leftBlinker
        nav_turn_eligible = nav_left or nav_right

    if not nav_turn_eligible or not _lat_active:
      self._nav_turn_nudged = False
    else:
      nudge_match = CS.steeringPressed and ((nav_left and CS.steeringTorque > 0.3) or (nav_right and CS.steeringTorque < -0.3))
      nudge_abort = CS.steeringPressed and ((nav_left and CS.steeringTorque < -0.3) or (nav_right and CS.steeringTorque > 0.3))
      if nudge_abort:
        self._nav_turn_nudged = False
      elif nudge_match:
        self._nav_turn_nudged = True

    nav_turn_confirmed = nav_turn_eligible and self._nav_turn_nudged

    blinker_pause = (CS.leftBlinker or CS.rightBlinker) and not nav_turn_confirmed
    CC.latActive = _lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   not blinker_pause and \
                   (not standstill or self.CP.steerAtStandstill)
    # custom-c4: lateral-only mode — disengage longitudinal control so the driver always controls throttle/brake
    CC.longActive = False

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits))

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    if self.sm.valid['lateralManeuverPlan']:
      new_desired_curvature = self.sm['lateralManeuverPlan'].desiredCurvature if CC.latActive else self.curvature
    else:
      new_desired_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature
    # custom-c4 conservative steering caps (original limits kept in comments for easy adjustment):
    #   Original MAX_LATERAL_ACCEL       = 3.6 m/s^2 (ISO_LATERAL_ACCEL = 3.0 m/s^2 + road bank)
    #   Original Tesla MAX_STEER_ANGLE   = 360.0 deg (flat, CarControllerParams.ANGLE_LIMITS)
    #   Original Tesla MAX_STEER_RATE    = 250.0 deg/s (5.0 deg per 20ms frame)
    # Speed-dependent steering angle cap derived from AASHTO Green Book min highway curve radii (e_max=6%)
    # for Tesla Model Y (L=2.89m, steerRatio=12.0) with ~25-40% lane-correction headroom:
    CONSERVATIVE_MAX_LAT_ACCEL = 3.6          # m/s^2 (original: ~3.6 m/s^2)
    # Speed-dependent steer rate cap: faster at low city speeds so 90 deg turns don't overshoot,
    # conservative 75 deg/s at >= 40 km/h (original flat Tesla limit: 250.0 deg/s)
    STEER_RATE_CAP_BP_KPH  = (0.0, 20.0, 30.0, 40.0)
    STEER_RATE_CAP_V_DEG_S = (200.0, 200.0, 120.0, 75.0)
    STEER_ANGLE_CAP_BP_KPH = (0.0, 30.0, 40.0, 50.0, 60.0, 80.0, 100.0, 120.0, 140.0)
    STEER_ANGLE_CAP_V_DEG  = (360.0, 360.0, 40.0, 28.0, 20.0, 14.0, 10.0,   7.0,   5.5)  # 0-30 km/h: default full (prev 50.0)

    def _interp_cap(v_kph: float, bp: tuple[float, ...], vals: tuple[float, ...]) -> float:
      if v_kph <= bp[0]:
        return vals[0]
      for i in range(len(bp) - 1):
        x0, x1 = bp[i], bp[i + 1]
        if v_kph <= x1:
          t = (v_kph - x0) / (x1 - x0)
          return vals[i] + t * (vals[i + 1] - vals[i])
      return vals[-1]

    if CC.latActive:
      v_eff = max(CS.vEgo, 1.0)
      max_curv = CONSERVATIVE_MAX_LAT_ACCEL / (v_eff ** 2)
      new_desired_curvature = max(-max_curv, min(max_curv, new_desired_curvature))

    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["lateralDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, lateral_output, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                     self.steer_limited_by_safety, self.desired_curvature,
                                                     self.calibrated_pose, curvature_limited, lat_delay)
    actuators.torque = float(steer)
    if self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      actuators.curvature = float(lateral_output)
    else:
      target_angle = float(lateral_output)
      if CC.latActive:
        v_kph = CS.vEgo * CV.MS_TO_KPH
        max_step = _interp_cap(v_kph, STEER_RATE_CAP_BP_KPH, STEER_RATE_CAP_V_DEG_S) * DT_CTRL
        max_angle = _interp_cap(v_kph, STEER_ANGLE_CAP_BP_KPH, STEER_ANGLE_CAP_V_DEG)
        target_angle = max(self._prev_steer_angle_deg - max_step, min(self._prev_steer_angle_deg + max_step, target_angle))
        target_angle = max(-max_angle, min(max_angle, target_angle))
        self._prev_steer_angle_deg = target_angle
      else:
        self._prev_steer_angle_deg = float(CS.steeringAngleDeg)
      actuators.steeringAngleDeg = target_angle
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.get_lat_active(self.sm):
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool(self.sm['driverMonitoringState'].noResponseForceDecel or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    # trigger the car's stock driver monitoring escalation
    CC.driverMonitoringEscalation = cs.forceDecel

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      cs.lateralControlState.curvatureState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      self.get_params_sp(self.sm)
      self.run_ext(self.sm, self.pm)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
