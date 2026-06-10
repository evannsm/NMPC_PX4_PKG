#!/usr/bin/env bash
# =============================================================================
# setup_px4_ros2_ws.sh
# -----------------------------------------------------------------------------
# Bootstraps a ROS 2 workspace for the PX4 controllers stack and links it to
# the PX4-ROS2-Docker repo so `make run` mounts it into the container.
#
# What it does (all steps are idempotent / safe to re-run):
#   1. Clones (or updates) PX4-ROS2-Docker into $DOCKER_DIR.
#   2. Creates the workspace at $WS_DIR/src.
#   3. Clones the controller source packages into $WS_DIR/src (skipping any
#      that already exist). Mirrors px4_ros2_controllers.repos.
#   4. "Links" the workspace to the Docker setup by symlinking the Makefile's
#      default WORKSPACE path (~/ws_px4_work) -> $WS_DIR, so `make run` with no
#      arguments mounts the right workspace.
#
# NOTE: px4_msgs is deliberately NOT cloned here. The Docker image pre-builds
#       it at /opt/ws_px4_msgs and sources it into the overlay; cloning it into
#       src/ would shadow that prebuilt copy.
#
# Usage:
#   ./setup_px4_ros2_ws.sh [--ws DIR] [--docker DIR] [--minimal] [--https]
#
#   --ws DIR       Workspace root        (default: ~/ros2px4_ws)
#   --docker DIR   PX4-ROS2-Docker clone (default: ~/PX4-ROS2-Docker)
#   --minimal      Clone only the packages NMPC needs (skips the other
#                  controllers: newton_raphson_px4, geometric_px4)
#   --https        Use https:// remotes instead of git@ SSH (default: SSH)
# =============================================================================
set -euo pipefail

# ── Defaults ────────────────────────────────────────────────────────────────
WS_DIR="${HOME}/ros2px4_ws"
DOCKER_DIR="${HOME}/PX4-ROS2-Docker"
DOCKER_URL_SSH="git@github.com:evannsmc/PX4-ROS2-Docker.git"
DOCKER_URL_HTTPS="https://github.com/evannsmc/PX4-ROS2-Docker.git"
DEFAULT_WORKSPACE_LINK="${HOME}/ws_px4_work"   # Makefile's default WORKSPACE
MINIMAL=0
USE_SSH=1
GH_OWNER="evannsmc"

# Package set, mirroring PX4-ROS2-Docker/px4_ros2_controllers.repos.
# Format: "dirname owner/repo required". required=1 packages are always cloned;
# required=0 (the extra controllers) are skipped under --minimal.
PACKAGES=(
  "quad_platforms      ${GH_OWNER}/quad_platforms      1"
  "quad_trajectories   ${GH_OWNER}/quad_trajectories   1"
  "ROS2Logger          ${GH_OWNER}/ROS2Logger          1"
  "nmpc_acados_px4      ${GH_OWNER}/nmpc_acados_px4      1"
  "newton_raphson_px4   ${GH_OWNER}/newton_raphson_px4   0"
  "geometric_px4        ${GH_OWNER}/geometric_px4        0"
)

# ── Tiny logging helpers ────────────────────────────────────────────────────
c_blue=$'\033[1;34m'; c_green=$'\033[1;32m'; c_yellow=$'\033[1;33m'; c_reset=$'\033[0m'
info()  { printf '%s==>%s %s\n' "$c_blue"   "$c_reset" "$*"; }
ok()    { printf '%s  ✓%s %s\n' "$c_green"  "$c_reset" "$*"; }
skip()  { printf '%s  ↷%s %s\n' "$c_yellow" "$c_reset" "$*"; }

# ── Parse args ──────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws)      WS_DIR="$2"; shift 2 ;;
    --docker)  DOCKER_DIR="$2"; shift 2 ;;
    --minimal) MINIMAL=1; shift ;;
    --https)   USE_SSH=0; shift ;;
    -h|--help) sed -n '2,33p' "$0"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done

# Expand a leading ~ that came in as a literal argument.
WS_DIR="${WS_DIR/#\~/$HOME}"
DOCKER_DIR="${DOCKER_DIR/#\~/$HOME}"

# Build a clone URL for owner/repo honouring the SSH/HTTPS choice.
repo_url() {
  local slug="$1"
  if [[ "$USE_SSH" -eq 1 ]]; then echo "git@github.com:${slug}.git"
  else echo "https://github.com/${slug}.git"; fi
}

command -v git >/dev/null || { echo "git is required but not found." >&2; exit 1; }

# ── 1. PX4-ROS2-Docker ──────────────────────────────────────────────────────
info "PX4-ROS2-Docker repo -> ${DOCKER_DIR}"
if [[ -d "${DOCKER_DIR}/.git" ]]; then
  skip "already cloned; pulling latest"
  git -C "${DOCKER_DIR}" pull --ff-only || skip "pull skipped (local changes or no upstream)"
else
  docker_url="${DOCKER_URL_SSH}"; [[ "$USE_SSH" -eq 0 ]] && docker_url="${DOCKER_URL_HTTPS}"
  git clone "${docker_url}" "${DOCKER_DIR}"
  ok "cloned PX4-ROS2-Docker"
fi

# ── 2. Workspace skeleton ───────────────────────────────────────────────────
info "Workspace -> ${WS_DIR}/src"
mkdir -p "${WS_DIR}/src"
ok "src/ ready"

# ── 3. Clone controller packages ────────────────────────────────────────────
info "Cloning controller packages into ${WS_DIR}/src"
for entry in "${PACKAGES[@]}"; do
  read -r dir slug required <<<"$entry"
  if [[ "$MINIMAL" -eq 1 && "$required" -eq 0 ]]; then
    skip "${dir} (skipped: --minimal)"
    continue
  fi
  target="${WS_DIR}/src/${dir}"
  if [[ -d "${target}/.git" ]]; then
    skip "${dir} (already present)"
  elif [[ -e "${target}" ]]; then
    skip "${dir} (path exists but is not a git repo — left untouched)"
  else
    git clone "$(repo_url "${slug}")" "${target}"
    ok "cloned ${dir}"
  fi
done

# ── 4. Link workspace to the Docker default WORKSPACE path ───────────────────
# `make run` (no args) mounts $(HOME)/ws_px4_work. Point it at our workspace so
# the container picks up the right src/ without needing WORKSPACE=... each time.
info "Linking Docker default WORKSPACE -> ${WS_DIR}"
if [[ "${DEFAULT_WORKSPACE_LINK}" == "${WS_DIR}" ]]; then
  skip "workspace already lives at the Makefile default; no link needed"
elif [[ -L "${DEFAULT_WORKSPACE_LINK}" ]]; then
  ln -sfn "${WS_DIR}" "${DEFAULT_WORKSPACE_LINK}"
  ok "updated symlink ${DEFAULT_WORKSPACE_LINK} -> ${WS_DIR}"
elif [[ -e "${DEFAULT_WORKSPACE_LINK}" ]]; then
  skip "${DEFAULT_WORKSPACE_LINK} exists and is not a symlink — not touching it"
  skip "run the container with: make -C ${DOCKER_DIR} run WORKSPACE=${WS_DIR}"
else
  ln -sn "${WS_DIR}" "${DEFAULT_WORKSPACE_LINK}"
  ok "created symlink ${DEFAULT_WORKSPACE_LINK} -> ${WS_DIR}"
fi

# ── Done — next steps ───────────────────────────────────────────────────────
cat <<EOF

${c_green}Workspace is set up and linked to PX4-ROS2-Docker.${c_reset}

Next steps:
  cd ${DOCKER_DIR}
  make build                       # build the Docker image (first time only)
  make run                         # start container (mounts ${WS_DIR})
  make build_ros                   # colcon build inside the container

For the NMPC controller you must compile the acados solver once inside the
container after build_ros:
  make attach
  cd /workspace/src/nmpc_acados_px4
  python3 nmpc_acados_px4_utils/controller/nmpc/generate_nmpc.py

Then run, e.g.:
  make ros2_run PKG=nmpc_acados_px4 EXEC=run_node ARGS="--platform sim --trajectory hover"
EOF
