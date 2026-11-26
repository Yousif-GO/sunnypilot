#!/usr/bin/env bash
# Install a persistent Cloudflare Tunnel for remote SSH on a comma four,
# and configure `ssh comma-remote` on this laptop.
#
# Prerequisites (one-time in Cloudflare Zero Trust dashboard):
#   1. Zero Trust -> Networks -> Tunnels -> Create a tunnel (Cloudflared)
#   2. Copy the tunnel token (starts with eyJ...)
#   3. Add a Public Hostname: e.g. ssh.yourdomain.com -> Service: SSH -> URL: localhost:22
#
# Usage (run from this laptop while connected to comma four on local Wi-Fi/hotspot):
#   ./tools/scripts/setup_comma_cloudflare_tunnel.sh <comma-local-ip> <tunnel-token> <ssh-hostname>
#
# Example:
#   ./tools/scripts/setup_comma_cloudflare_tunnel.sh 10.124.11.89 eyJhIjoi... comma.yourdomain.com
set -euo pipefail

if [[ $# -lt 3 ]]; then
  echo "Usage: $0 <comma-local-ip> <cloudflare-tunnel-token> <ssh-hostname>"
  echo "Example: $0 10.124.11.89 eyJhIjoi... comma.yourdomain.com"
  exit 1
fi

COMMA_IP="$1"
CF_TOKEN="$2"
CF_HOST="$3"
SSH_KEY="${HOME}/.ssh/id_ed25519"
CLOUDFLARED_BIN="${HOME}/.local/bin/cloudflared"

if [[ ! -x "$CLOUDFLARED_BIN" ]]; then
  echo "Installing cloudflared on this laptop..."
  mkdir -p "${HOME}/.local/bin"
  curl -fsSL https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64 -o "$CLOUDFLARED_BIN"
  chmod +x "$CLOUDFLARED_BIN"
fi

echo "[1/3] Installing persistent cloudflared (arm64) on comma@${COMMA_IP}..."
ssh -i "$SSH_KEY" -o StrictHostKeyChecking=accept-new "comma@${COMMA_IP}" "bash -s" -- "$CF_TOKEN" << 'REMOTE_EOF'
set -euo pipefail
TOKEN="$1"

mkdir -p /data/cloudflared
if [[ ! -x /data/cloudflared/cloudflared ]]; then
  curl --retry 5 --retry-delay 2 -fsSL \
    https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64 \
    -o /data/cloudflared/cloudflared
  chmod +x /data/cloudflared/cloudflared
fi

printf '%s' "$TOKEN" > /data/cloudflared/token
chmod 600 /data/cloudflared/token

cat << 'EOF' > /data/cloudflared/start.sh
#!/usr/bin/env bash
if ! pgrep -f "/data/cloudflared/cloudflared" >/dev/null 2>&1; then
  nohup /data/cloudflared/cloudflared tunnel --no-autoupdate --protocol http2 run --token "$(cat /data/cloudflared/token)" \
    >/data/cloudflared/cloudflared.log 2>&1 &
fi
EOF
chmod +x /data/cloudflared/start.sh

if ! grep -q "/data/cloudflared/start.sh" /data/continue.sh; then
  sed -i '/^cd \/data\/openpilot/i /data/cloudflared/start.sh' /data/continue.sh
fi

pkill -f "/data/cloudflared/cloudflared" 2>/dev/null || true
/data/cloudflared/start.sh
sleep 3
pgrep -a cloudflared
/data/cloudflared/cloudflared --version
REMOTE_EOF

echo "[2/3] Configuring ~/.ssh/config on this laptop for 'ssh comma-remote'..."
mkdir -p "${HOME}/.ssh"
touch "${HOME}/.ssh/config"
chmod 600 "${HOME}/.ssh/config"

# Remove any previous comma-remote block cleanly
python3 - "$CF_HOST" "$SSH_KEY" "$CLOUDFLARED_BIN" << 'PY_EOF'
import os, re, sys
cf_host, ssh_key, cf_bin = sys.argv[1], sys.argv[2], sys.argv[3]
cfg_path = os.path.expanduser("~/.ssh/config")
text = open(cfg_path).read() if os.path.exists(cfg_path) else ""
text = re.sub(r"\n?# BEGIN comma-remote\n.*?# END comma-remote\n?", "\n", text, flags=re.S)
block = f"""
# BEGIN comma-remote
Host comma-remote
  HostName {cf_host}
  User comma
  IdentityFile {ssh_key}
  ProxyCommand {cf_bin} access ssh --hostname %h
  ServerAliveInterval 30
  ServerAliveCountMax 3
# END comma-remote
"""
with open(cfg_path, "w") as f:
  f.write(text.rstrip() + "\n" + block)
PY_EOF

echo "[3/3] Testing remote SSH over Cloudflare Tunnel..."
ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=15 comma-remote "hostname && echo 'REMOTE_SSH_OK'"

echo ""
echo "All set! You can now SSH from ANY Wi-Fi network with:"
echo "  ssh comma-remote"
echo "And pull recordings remotely with:"
echo "  ./tools/scripts/pull_comma_recordings.sh comma-remote ./comma_recordings --mp4"
