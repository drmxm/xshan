#!/usr/bin/env bash
# xshan/rebuild — fresh build & run sensors+infra; verifies Foxglove WS
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
COMPOSE_DIR="$ROOT/deploy/compose"
COMPOSE="$COMPOSE_DIR/docker-compose.yml"

PROFILES=(sensors infra)  # default
PORT="${FOXGLOVE_PORT:-8765}"
DEVICE="${CAMERA_DEVICE:-/dev/video0}"

usage(){ echo "usage: $0 [-p|--profiles sensors,infra]"; }

# ---- parse args (correct case/esac + shifting) ----
while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--profiles)
      [[ -n "${2:-}" ]] || { echo "error: --profiles needs a value" >&2; usage; exit 1; }
      IFS=',' read -r -a PROFILES <<< "$2"
      shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown arg: $1" >&2; usage; exit 1 ;;
  esac
done

# preflight
[[ -e "$DEVICE" ]] || { echo "[rebuild] camera device $DEVICE not found" >&2; exit 1; }
[[ -d /usr/lib/aarch64-linux-gnu/tegra ]] || { echo "[rebuild] tegra plugin dir missing on host" >&2; exit 1; }

cd "$COMPOSE_DIR"

# build compose --profile args
compose_profiles_args=()
for p in "${PROFILES[@]}"; do compose_profiles_args+=(--profile "$p"); done

echo "[rebuild] build (no cache, pull)…"
docker compose -f "$COMPOSE" "${compose_profiles_args[@]}" build --no-cache --pull

echo "[rebuild] up (force recreate)…"
docker compose -f "$COMPOSE" "${compose_profiles_args[@]}" up -d --force-recreate --remove-orphans

# quick WS check if infra is in profiles
if printf '%s\n' "${PROFILES[@]}" | grep -qx infra; then
  sleep 2
  if command -v nc >/dev/null 2>&1 && nc -z 127.0.0.1 "$PORT"; then
    echo "[rebuild] foxglove ws is listening at ws://localhost:$PORT"
  else
    echo "[rebuild] note: ws://localhost:$PORT not open yet; infra may still be starting"
  fi
fi

echo "[rebuild] done. open Foxglove → Foxglove WebSocket → ws://localhost:$PORT"
