#!/usr/bin/env bash
# Pull all recorded video (fcamera/ecamera/dcamera.hevc) and vehicle telemetry
# (rlog.zst / qlog.zst) directly from a comma four over SSH, bypassing sunnylink.
#
# Usage:
#   ./tools/scripts/pull_comma_recordings.sh <comma-ip> [dest-dir] [--mp4]
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <comma-ip> [dest-dir] [--mp4]"
  echo "Example: $0 192.168.1.50 ./comma_recordings --mp4"
  exit 1
fi

COMMA_TARGET="$1"
DEST_DIR="${2:-./comma_recordings}"
CONVERT_MP4=0
for arg in "$@"; do
  if [[ "$arg" == "--mp4" ]]; then
    CONVERT_MP4=1
  fi
done

mkdir -p "$DEST_DIR"

if [[ "$COMMA_TARGET" == "comma-remote" || "$COMMA_TARGET" == *"@"* ]]; then
  REMOTE_SPEC="${COMMA_TARGET}:/data/media/0/realdata/"
  SSH_CMD="ssh"
else
  REMOTE_SPEC="comma@${COMMA_TARGET}:/data/media/0/realdata/"
  SSH_CMD="ssh -i ~/.ssh/id_ed25519"
fi

echo "Pulling all video & telemetry from ${REMOTE_SPEC} -> ${DEST_DIR}"
rsync -avz --progress \
  -e "$SSH_CMD" \
  "$REMOTE_SPEC" \
  "${DEST_DIR}/"

if [[ "$CONVERT_MP4" -eq 1 ]]; then
  if command -v ffmpeg >/dev/null 2>&1; then
    echo "Converting .hevc segments to .mp4..."
    find "$DEST_DIR" -name "*.hevc" | while read -r hevc; do
      mp4="${hevc%.hevc}.mp4"
      if [[ ! -f "$mp4" ]]; then
        ffmpeg -loglevel error -r 20 -i "$hevc" -c:v copy "$mp4"
      fi
    done
  else
    echo "ffmpeg not found; skipping .mp4 conversion."
  fi
fi

echo "Done! Recordings saved in: $DEST_DIR"
echo "To replay a route locally:"
echo "  source .venv/bin/activate"
echo "  openpilot/tools/replay/replay --data_dir \"$DEST_DIR\" <route_name>"
