#!/usr/bin/env bash
# Build, and optionally flash, the Crazyflie firmware with the WedgeBug app.
#
#   apps/wedgebug/build.sh                 build -> apps/wedgebug/build/cf2.bin
#   apps/wedgebug/build.sh flash [URI]     build, then flash it over the Crazyradio
#                                          (default URI radio://0/80/2M/E7E7E7E701)
#
# The firmware's Kbuild refuses to build in a directory whose path contains a
# space or a colon. When this repository's path has one, the repository
# (including .git, which the version stamp reads) is mirrored with rsync to
# $WEDGEBUG_BUILD_DIR (default ~/.cache/wedgebug-build) and built there; the
# result is copied back to apps/wedgebug/build/cf2.bin. Later runs only copy
# what changed. Without a space in the path this is plain `make` in apps/wedgebug.
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$APP_DIR/../.." && pwd)"
DEFAULT_URI="radio://0/80/2M/E7E7E7E701"

case "${1:-build}" in
  build|flash) ;;
  -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
  *) echo "usage: $0 [build | flash [URI]]" >&2; exit 2 ;;
esac

if [[ "$REPO" == *" "* || "$REPO" == *:* ]]; then
  BUILD_REPO="${WEDGEBUG_BUILD_DIR:-$HOME/.cache/wedgebug-build}"
  if [[ "$BUILD_REPO" == *" "* || "$BUILD_REPO" == *:* ]]; then
    echo "WEDGEBUG_BUILD_DIR must not contain spaces or colons: $BUILD_REPO" >&2
    exit 1
  fi
  echo "Repository path has a space; building in a mirror at $BUILD_REPO"
  mkdir -p "$BUILD_REPO"
  # Excluded paths are neither copied nor deleted, so the mirror's own build
  # output survives between runs.
  rsync -a --delete \
    --exclude='/build/' --exclude='/apps/wedgebug/build/' --exclude='/apps/wedgebug/test/build/' \
    --exclude='/sim/' --exclude='/matlab-trajectory-sim/' --exclude='/docs/' --exclude='/real/' \
    "$REPO/" "$BUILD_REPO/"
else
  BUILD_REPO="$REPO"
fi

# oot.mk takes the app folder from $PWD, so build from inside it.
(cd "$BUILD_REPO/apps/wedgebug" && make -j"$(nproc)")

BIN="$APP_DIR/build/cf2.bin"
if [[ "$BUILD_REPO" != "$REPO" ]]; then
  mkdir -p "$APP_DIR/build"
  cp "$BUILD_REPO/apps/wedgebug/build/cf2.bin" "$BIN"
fi
echo "Firmware: $BIN"

if [[ "${1:-build}" == "flash" ]]; then
  URI="${2:-$DEFAULT_URI}"
  if command -v cfloader >/dev/null; then
    LOADER=(cfloader)
  else
    LOADER=(python3 -m cfloader)
  fi
  echo "Flashing over $URI (the Crazyflie must be on, running firmware, with nothing else using the radio)"
  # -w: warm boot, reset the running Crazyflie into its bootloader over the radio.
  "${LOADER[@]}" flash "$BIN" stm32-fw -w "$URI"
fi
