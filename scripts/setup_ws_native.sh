#!/usr/bin/env bash
# =============================================================================
# setup_ws_native.sh
# -----------------------------------------------------------------------------
# Sets up a native (Docker-free) ROS 2 workspace for this controller: it lays
# out <ws>/src with every sibling package AND px4_msgs, then optionally runs
# colcon build. Use this if you build/run directly on the host instead of in
# the PX4-ROS2-Docker container (for the Docker path, use setup_ws_docker.sh).
#
# What it does (idempotent / safe to re-run):
#   1. Creates the workspace at $WS_DIR/src.
#   2. Clones the sibling source packages + px4_msgs into $WS_DIR/src,
#      skipping any that already exist.
#   3. If run from inside this controller's own git checkout, symlinks that
#      checkout into the workspace instead of re-cloning it.
#   4. Optionally (--build) runs `colcon build --symlink-install`.
#
# Unlike the Docker workflow, px4_msgs IS cloned here — there is no prebuilt
# image overlay to provide it.
#
# Usage:
#   ./setup_ws_native.sh [--ws DIR] [--minimal] [--https] [--build]
#
#   --ws DIR     Workspace root                          (default: ~/ros2px4_ws)
#   --minimal    Clone only what this controller needs (skips the other
#                controllers: newton_raphson_px4, geometric_px4)
#   --https      Use https:// remotes instead of git@ SSH (default: SSH)
#   --build      Run `colcon build --symlink-install` after cloning
# =============================================================================
set -euo pipefail

# ── Defaults ────────────────────────────────────────────────────────────────
WS_DIR="${HOME}/ros2px4_ws"
THIS_PKG="nmpc_acados_px4"
MINIMAL=0
USE_SSH=1
DO_BUILD=0

# Package set. Format: "dirname owner/repo required".
# px4_msgs comes from the PX4 org and is always cloned for native builds.
PACKAGES=(
  "quad_platforms      evannsmc/quad_platforms   1"
  "quad_trajectories   evannsmc/quad_trajectories 1"
  "ROS2Logger          evannsmc/ROS2Logger       1"
  "nmpc_acados_px4      evannsmc/nmpc_acados_px4   1"
  "px4_msgs            PX4/px4_msgs              1"
  "newton_raphson_px4   evannsmc/newton_raphson_px4 0"
  "geometric_px4        evannsmc/geometric_px4    0"
)

# ── Logging helpers ─────────────────────────────────────────────────────────
c_blue=$'\033[1;34m'; c_green=$'\033[1;32m'; c_yellow=$'\033[1;33m'; c_reset=$'\033[0m'
info() { printf '%s==>%s %s\n' "$c_blue"   "$c_reset" "$*"; }
ok()   { printf '%s  ✓%s %s\n' "$c_green"  "$c_reset" "$*"; }
skip() { printf '%s  ↷%s %s\n' "$c_yellow" "$c_reset" "$*"; }

# ── Parse args ──────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws)      WS_DIR="$2"; shift 2 ;;
    --minimal) MINIMAL=1; shift ;;
    --https)   USE_SSH=0; shift ;;
    --build)   DO_BUILD=1; shift ;;
    -h|--help) sed -n '2,33p' "$0"; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done

WS_DIR="${WS_DIR/#\~/$HOME}"

# git@ for our repos; PX4 org is public, prefer https unless SSH explicitly fine.
repo_url() {
  local slug="$1"
  if [[ "${slug%%/*}" == "PX4" ]]; then echo "https://github.com/${slug}.git"; return; fi
  if [[ "$USE_SSH" -eq 1 ]]; then echo "git@github.com:${slug}.git"
  else echo "https://github.com/${slug}.git"; fi
}

command -v git >/dev/null || { echo "git is required but not found." >&2; exit 1; }

# Where is this script's own controller checkout (if any)? Used to symlink the
# current repo into the workspace rather than cloning a second copy.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SELF_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null || true)"

# ── 1. Workspace skeleton ───────────────────────────────────────────────────
info "Workspace -> ${WS_DIR}/src"
mkdir -p "${WS_DIR}/src"
ok "src/ ready"

# ── 2. Populate src/ ────────────────────────────────────────────────────────
info "Populating ${WS_DIR}/src"
for entry in "${PACKAGES[@]}"; do
  read -r dir slug required <<<"$entry"
  if [[ "$MINIMAL" -eq 1 && "$required" -eq 0 ]]; then
    skip "${dir} (skipped: --minimal)"
    continue
  fi
  target="${WS_DIR}/src/${dir}"
  if [[ -e "${target}" || -L "${target}" ]]; then
    skip "${dir} (already present)"
    continue
  fi
  # Reuse the current checkout for this package instead of re-cloning it.
  if [[ "${dir}" == "${THIS_PKG}" && -n "${SELF_ROOT}" \
        && "$(basename "${SELF_ROOT}")" == "${THIS_PKG}" \
        && "${SELF_ROOT}" != "${target}" ]]; then
    ln -sn "${SELF_ROOT}" "${target}"
    ok "linked ${dir} -> ${SELF_ROOT} (current checkout)"
  else
    git clone "$(repo_url "${slug}")" "${target}"
    ok "cloned ${dir}"
  fi
done

# ── 3. Optional build ───────────────────────────────────────────────────────
if [[ "$DO_BUILD" -eq 1 ]]; then
  info "Building workspace with colcon"
  if ! command -v colcon >/dev/null; then
    skip "colcon not found — skipping build (is ROS 2 installed & sourced?)"
  elif [[ -z "${ROS_DISTRO:-}" ]]; then
    skip "ROS 2 not sourced (ROS_DISTRO unset) — run 'source /opt/ros/<distro>/setup.bash' first"
  else
    ( cd "${WS_DIR}" && colcon build --symlink-install )
    ok "colcon build complete"
  fi
fi

# ── Done — next steps ───────────────────────────────────────────────────────
cat <<EOF

${c_green}Native workspace is laid out at ${WS_DIR}.${c_reset}

Before building, install the controller's external deps that are NOT ROS
packages:
  • acados + acados_template  — see the "Acados Setup" section of the README
  • SciPy                     — pip install scipy

Then build and source:
  cd ${WS_DIR}
  colcon build --symlink-install        # (or re-run this script with --build)
  source install/setup.bash

Run, e.g.:
  ros2 run nmpc_acados_px4 run_node --platform sim --trajectory hover
EOF
