#!/usr/bin/env bash
set -euo pipefail

COMPOSE_FILE="${COMPOSE_FILE:-docker-compose.yml}"
SERVICE="rosdev"
DO_COLCON="${1:-}"

cd "$(dirname "$0")"

# Remove any stale container
if docker ps -a --format '{{.Names}}' | grep -qx "${SERVICE}"; then
  echo "[run_rosdev] Removing existing container '${SERVICE}'..."
  docker rm -f "${SERVICE}" >/dev/null
fi

echo "[run_rosdev] Building image…"
docker compose -f "${COMPOSE_FILE}" build

echo "[run_rosdev] Starting container…"
docker compose -f "${COMPOSE_FILE}" up -d

# Attach
if [[ "${DO_COLCON}" == "--colcon" ]]; then
  echo "[run_rosdev] Attaching (with colcon build)…"
  docker exec -it "${SERVICE}" bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    cd /root/ros2_ws
    if [ -d src ]; then
      rm -rf build install log || true
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
      source install/setup.bash
    fi
    exec bash
  '
else
  echo "[run_rosdev] Attaching…"
  docker exec -it "${SERVICE}" bash
fi
