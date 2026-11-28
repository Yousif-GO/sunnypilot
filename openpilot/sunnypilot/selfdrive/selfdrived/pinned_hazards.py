"""
Phase 1 Pinned Hazards Shadow Matcher (HUD Alerts Only — Zero Longitudinal Actuation).

Loads offline OSM + inferred hazards (/data/comma_web_ui/osm_hazards.json) and
user manual pins + mutes (/data/comma_web_ui/pinned_hazards.json) into a fast
spatial hash grid (~110 m cells) and checks for upcoming speed bumps and
directional stop signs along the vehicle's current GPS heading.
"""

from collections import defaultdict
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

OSM_HAZARDS_PATH = Path("/data/comma_web_ui/osm_hazards.json")
PINNED_HAZARDS_PATH = Path("/data/comma_web_ui/pinned_hazards.json")

ACTIVE_TYPES = {"speed_bump", "stop_sign", "inferred_stop", "sharp_curve"}
DEG_TO_RAD = math.pi / 180.0
EARTH_M_PER_DEG_LAT = 111320.0


@dataclass(frozen=True)
class HazardMatch:
  hazard_id: str
  hazard_type: str
  source: str
  along_m: float
  cross_m: float
  target_speed_kmh: int
  req_decel_ms2: float
  is_new_trigger: bool


def _angle_diff_deg(a: float, b: float) -> float:
  d = abs(a - b) % 360.0
  return 360.0 - d if d > 180.0 else d


class PinnedHazardMatcher:
  def __init__(self, osm_path: Path = OSM_HAZARDS_PATH, pin_path: Path = PINNED_HAZARDS_PATH):
    self.osm_path = osm_path
    self.pin_path = pin_path
    self._grid: dict[tuple[int, int], list[dict]] = defaultdict(list)
    self._osm_mtime = 0.0
    self._pin_mtime = 0.0
    self._last_reload_check_t = 0.0
    self._last_alerted_id: str | None = None
    self._last_alerted_t: dict[str, float] = {}
    self._last_match: HazardMatch | None = None
    self._last_eval_t = 0.0
    self.total_loaded = 0
    self._reload_if_needed(force=True)

  def _reload_if_needed(self, force: bool = False) -> None:
    now = time.monotonic()
    if not force and (now - self._last_reload_check_t) < 5.0:
      return
    self._last_reload_check_t = now

    osm_m = self.osm_path.stat().st_mtime if self.osm_path.is_file() else 0.0
    pin_m = self.pin_path.stat().st_mtime if self.pin_path.is_file() else 0.0
    if not force and osm_m == self._osm_mtime and pin_m == self._pin_mtime:
      return

    self._osm_mtime = osm_m
    self._pin_mtime = pin_m
    grid: dict[tuple[int, int], list[dict]] = defaultdict(list)
    muted: set[str] = set()
    manual_pins: list[dict] = []

    if pin_m > 0.0:
      try:
        pdata = json.loads(self.pin_path.read_text(encoding="utf-8"))
        muted = set(pdata.get("muted_osm_ids", []))
        manual_pins = pdata.get("pins", [])
      except Exception:
        pass

    count = 0
    # Load manual pins first (highest priority)
    for item in manual_pins:
      if item.get("type") not in ACTIVE_TYPES:
        continue
      lat = item.get("lat")
      lon = item.get("lon")
      if lat is None or lon is None:
        continue
      grid[(int(lat * 1000), int(lon * 1000))].append(item)
      count += 1

    # Load offline OSM + inferred hazards (skip muted)
    if osm_m > 0.0:
      try:
        odata = json.loads(self.osm_path.read_text(encoding="utf-8"))
        for item in odata.get("hazards", []):
          if item.get("type") not in ACTIVE_TYPES:
            continue
          if item.get("id") in muted:
            continue
          lat = item.get("lat")
          lon = item.get("lon")
          if lat is None or lon is None:
            continue
          grid[(int(lat * 1000), int(lon * 1000))].append(item)
          count += 1
      except Exception:
        pass

    self._grid = grid
    self.total_loaded = count

  def update(self, lat: float, lon: float, bearing_deg: float, v_ego_ms: float) -> HazardMatch | None:
    now = time.monotonic()
    self._reload_if_needed(force=False)

    # Evaluate at 5 Hz (every 0.2s) to keep CPU negligible
    if (now - self._last_eval_t) < 0.2:
      if self._last_match is not None:
        return HazardMatch(
          self._last_match.hazard_id,
          self._last_match.hazard_type,
          self._last_match.source,
          self._last_match.along_m,
          self._last_match.cross_m,
          self._last_match.target_speed_kmh,
          self._last_match.req_decel_ms2,
          False,
        )
      return None
    self._last_eval_t = now

    if self.total_loaded == 0 or v_ego_ms < 3.0 or abs(lat) < 1.0 or abs(lon) < 1.0:
      self._last_match = None
      return None

    cos_lat = math.cos(lat * DEG_TO_RAD)
    m_per_deg_lon = EARTH_M_PER_DEG_LAT * cos_lat
    b_rad = bearing_deg * DEG_TO_RAD
    fwd_n = math.cos(b_rad)
    fwd_e = math.sin(b_rad)

    gx, gy = int(lat * 1000), int(lon * 1000)
    best: HazardMatch | None = None
    v_kmh = v_ego_ms * 3.6

    for dx in (-1, 0, 1):
      for dy in (-1, 0, 1):
        for h in self._grid.get((gx + dx, gy + dy), ()):
          htype = h["type"]
          target_kmh = int(h.get("target_speed_kmh", 0 if "stop" in htype else 15))
          # Speed bumps: only alert if driving noticeably faster than bump target speed
          if htype == "speed_bump" and v_kmh <= (target_kmh + 5.0):
            continue

          # Stop signs (OSM, inferred, manual): strictly one-way, require known heading within ±35°
          if htype in ("stop_sign", "inferred_stop"):
            h_hdg = h.get("heading_deg")
            if h_hdg is None:
              continue
            if _angle_diff_deg(bearing_deg, float(h_hdg)) > 35.0:
              continue

          dn = (h["lat"] - lat) * EARTH_M_PER_DEG_LAT
          de = (h["lon"] - lon) * m_per_deg_lon
          along_m = dn * fwd_n + de * fwd_e
          cross_m = abs(de * fwd_n - dn * fwd_e)

          lead_m = float(h.get("lead_dist_m", 45.0))
          lookahead_m = min(120.0, max(35.0, lead_m, v_ego_ms * 4.5))
          max_cross_m = 8.0 if htype in ("stop_sign", "inferred_stop") else 10.0

          if not (3.0 <= along_m <= lookahead_m and cross_m <= max_cross_m):
            continue

          if best is None or along_m < best.along_m:
            v_tgt_ms = target_kmh / 3.6
            req_decel = max(0.0, (v_ego_ms * v_ego_ms - v_tgt_ms * v_tgt_ms) / (2.0 * max(along_m, 1.0)))
            hid = str(h.get("id", ""))
            last_t = self._last_alerted_t.get(hid, -1e9)
            is_new = (hid != self._last_alerted_id) and ((now - last_t) > 12.0)
            best = HazardMatch(
              hazard_id=hid,
              hazard_type=htype,
              source=str(h.get("source", "osm")),
              along_m=along_m,
              cross_m=cross_m,
              target_speed_kmh=target_kmh,
              req_decel_ms2=req_decel,
              is_new_trigger=is_new,
            )

    if best is not None:
      if best.is_new_trigger:
        self._last_alerted_t[best.hazard_id] = now
      self._last_alerted_id = best.hazard_id
    else:
      self._last_alerted_id = None

    self._last_match = best
    return best
