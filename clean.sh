#!/usr/bin/env bash
# xshan/clean — stop & remove xshan containers; optional: images, caches, volumes
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
COMPOSE="$ROOT/deploy/compose/docker-compose.yml"

ALL=false        # also remove images + caches
VOLUMES=false    # also remove volumes (down + prune)
PROFILES=(sensors infra)

usage(){ echo "usage: $0 [-a|--all] [-v|--volumes] [-p|--profiles sensors,infra]"; }

# ---- parse args (correct case/esac + shifting) ----
while [[ $# -gt 0 ]]; do
  case "$1" in
    -a|--all) ALL=true; shift ;;
    -v|--volumes) VOLUMES=true; shift ;;
    -p|--profiles)
      [[ -n "${2:-}" ]] || { echo "error: --profiles needs a value" >&2; usage; exit 1; }
      IFS=',' read -r -a PROFILES <<< "$2"
      shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown arg: $1" >&2; usage; exit 1 ;;
  esac
done

# build compose --profile args
compose_profiles_args=()
for p in "${PROFILES[@]}"; do compose_profiles_args+=(--profile "$p"); done

# down (don’t fail if already stopped)
docker compose -f "$COMPOSE" "${compose_profiles_args[@]}" down --remove-orphans $($VOLUMES && echo --volumes) || true

if $ALL; then
  # remove only images referenced by this compose (respecting profiles)
  mapfile -t IMAGES < <(docker compose -f "$COMPOSE" "${compose_profiles_args[@]}" config --images | sort -u || true)
  if ((${#IMAGES[@]})); then docker rmi -f "${IMAGES[@]}" 2>/dev/null || true; fi
  # prune caches
  docker builder prune -a -f || true
  if $VOLUMES; then docker system prune -a -f --volumes || true
  else docker system prune -a -f || true; fi
fi

echo "[clean] done."
