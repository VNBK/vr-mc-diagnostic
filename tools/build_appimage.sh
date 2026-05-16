#!/usr/bin/env bash
# build_appimage.sh — package the motor-control diagnostic GUI as a
# portable AppImage. Modelled on vr-hand-diagnostic/tools/build_appimage.sh
# but adapted for the vr-mc-diagnostic layout (pure CMake build tree,
# in-tree simulator binary, vendor ZLG .so).
#
# Layout produced inside the AppImage:
#
#   /usr/bin/vr_mc_diagnostic                        (the GUI binary)
#   /usr/bin/vrmc_sim                                (in-tree CiA-402 sim)
#   /usr/lib/libcontrolcanfd.so                      (ZLG USB-CANFD driver)
#   /usr/lib/lib*.so                                 (Qt + transitive deps)
#   /usr/share/vr_mc_diagnostic/docs/...             (user guide HTML/PDF/MD/images)
#   /usr/share/applications/vr_mc_diagnostic.desktop
#   /usr/share/icons/hicolor/256x256/apps/vr_mc_diagnostic.png
#
# Usage:
#   tools/build_appimage.sh [VERSION]
#
# Output:
#   dist/VinRobotics_MotorControl_Diagnostic_Tool-<version>-<arch>.AppImage
#
# Caveats:
#   - The AppImage has *both* binaries inside. Help → Start demo finds
#     vrmc_sim via the same dir as vr_mc_diagnostic and spawns it.
#   - libcontrolcanfd.so is the vendor's prebuilt ZLG driver. The
#     diagnostic + sim both transitively link against it via the SDK's
#     platform_linux target. Without it the binaries fail to load.
#   - Needs libfuse2 to mount itself; fall back to
#         /path/to/<app>.AppImage --appimage-extract-and-run
#     on systems without FUSE.

set -euo pipefail

# -- locate workspace + repo paths ----------------------------------------
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DIAG_PKG="${ROOT_DIR}/vr-mc-diagnostic"
SDK_PKG="${ROOT_DIR}/vr-mc-sdk"
TOOLS_CACHE="${DIAG_PKG}/tools/.cache"

VERSION="${1:-1.0.0}"
ARCH="$(uname -m)"
DIST="${ROOT_DIR}/dist"
APPDIR="${DIST}/AppDir"

# -- inputs ---------------------------------------------------------------
# Pure cmake layout (no colcon), so binaries live in build/. Run
# `cmake --build vr-mc-diagnostic/build` first if these are missing.
DIAG_BIN="${DIAG_PKG}/build/vr_mc_diagnostic"
SIM_BIN="${DIAG_PKG}/build/vrmc_sim"
DOCS_DIR="${DIAG_PKG}/docs"
ICON_SRC="${DIAG_PKG}/resources/vinrobotic.png"

# Vendor ZLG driver — picked by arch to match the AppImage host.
case "${ARCH}" in
    aarch64|arm64) ZLG_LIB="${SDK_PKG}/src/platform/linux/zlg/lib/libcontrolcanfd_arm64.so" ;;
    arm*|armv7*)   ZLG_LIB="${SDK_PKG}/src/platform/linux/zlg/lib/libcontrolcanfd_arm32.so" ;;
    *)             ZLG_LIB="${SDK_PKG}/src/platform/linux/zlg/lib/libcontrolcanfd.so" ;;
esac

for f in "${DIAG_BIN}" "${SIM_BIN}" "${ZLG_LIB}" "${ICON_SRC}"; do
    [[ -e "${f}" ]] || { echo "ERROR: missing ${f}"; \
        echo "Build first:  cmake --build vr-mc-diagnostic/build"; exit 1; }
done

# -- linuxdeploy + the qt plugin ------------------------------------------
mkdir -p "${TOOLS_CACHE}"
LDP="${TOOLS_CACHE}/linuxdeploy-${ARCH}.AppImage"
LDP_QT="${TOOLS_CACHE}/linuxdeploy-plugin-qt-${ARCH}.AppImage"

if [[ ! -x "${LDP}" ]]; then
    echo "downloading linuxdeploy (one-time)..."
    curl -fL --retry 3 -o "${LDP}" \
        "https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-${ARCH}.AppImage"
    chmod +x "${LDP}"
fi
if [[ ! -x "${LDP_QT}" ]]; then
    echo "downloading linuxdeploy-plugin-qt (one-time)..."
    curl -fL --retry 3 -o "${LDP_QT}" \
        "https://github.com/linuxdeploy/linuxdeploy-plugin-qt/releases/download/continuous/linuxdeploy-plugin-qt-${ARCH}.AppImage"
    chmod +x "${LDP_QT}"
fi

# linuxdeploy looks up plugins by basename ("linuxdeploy-plugin-qt") via PATH.
ln -sf "linuxdeploy-plugin-qt-${ARCH}.AppImage" "${TOOLS_CACHE}/linuxdeploy-plugin-qt"
export PATH="${TOOLS_CACHE}:${PATH}"

# Run linuxdeploy without FUSE so this script works in containers /
# minimal environments where /dev/fuse isn't around.
LDP_RUN=("${LDP}" --appimage-extract-and-run)

# Force Qt6 qmake (linuxdeploy-plugin-qt defaults to qmake5 on Debian).
QMAKE_BIN="${QMAKE:-$(command -v qmake6 || true)}"
if [[ -z "${QMAKE_BIN}" ]]; then
    for cand in /usr/lib/qt6/bin/qmake6 /usr/lib/qt6/bin/qmake \
                /usr/lib/x86_64-linux-gnu/qt6/bin/qmake6; do
        [[ -x "${cand}" ]] && { QMAKE_BIN="${cand}"; break; }
    done
fi
[[ -x "${QMAKE_BIN}" ]] || { echo "ERROR: Qt6 qmake not found — install qt6-base-dev-tools"; exit 1; }
export QMAKE="${QMAKE_BIN}"
echo "using Qt6 qmake: ${QMAKE}"

# -- stage AppDir ---------------------------------------------------------
rm -rf "${APPDIR}"
mkdir -p \
    "${APPDIR}/usr/bin" \
    "${APPDIR}/usr/lib" \
    "${APPDIR}/usr/share/vr_mc_diagnostic/docs" \
    "${APPDIR}/usr/share/applications" \
    "${APPDIR}/usr/share/icons/hicolor/256x256/apps"

cp -L "${DIAG_BIN}" "${APPDIR}/usr/bin/vr_mc_diagnostic"
cp -L "${SIM_BIN}"  "${APPDIR}/usr/bin/vrmc_sim"

# Vendor ZLG driver. Both binaries pick it up via RPATH or the
# AppImage loader's library search path. Strip the arch suffix on
# arm builds so the SONAME the diagnostic was linked against matches.
cp -L "${ZLG_LIB}" "${APPDIR}/usr/lib/libcontrolcanfd.so"

# Docs — Help → Documentation reads from /usr/share/vr_mc_diagnostic/docs.
if [[ -d "${DOCS_DIR}" ]]; then
    cp -r "${DOCS_DIR}/." "${APPDIR}/usr/share/vr_mc_diagnostic/docs/"
fi

# Branding — resize 225×225 PNG to a canonical 256×256.
ICON_DST="${APPDIR}/usr/share/icons/hicolor/256x256/apps/vr_mc_diagnostic.png"
if command -v convert >/dev/null 2>&1; then
    convert "${ICON_SRC}" -resize 256x256 "${ICON_DST}"
elif python3 -c 'import PIL' 2>/dev/null; then
    python3 - <<PY
from PIL import Image
img = Image.open("${ICON_SRC}").convert("RGBA")
img.thumbnail((256, 256))
out = Image.new("RGBA", (256, 256), (0, 0, 0, 0))
out.paste(img, ((256 - img.width)//2, (256 - img.height)//2))
out.save("${ICON_DST}")
PY
elif command -v magick >/dev/null 2>&1; then
    magick "${ICON_SRC}" -resize 256x256 "${ICON_DST}"
else
    echo "WARNING: no ImageMagick / PIL — copying icon at native 225×225 size;"
    echo "         linuxdeploy may reject it. Install:  sudo apt install imagemagick"
    cp "${ICON_SRC}" "${ICON_DST}"
fi

cat > "${APPDIR}/usr/share/applications/vr_mc_diagnostic.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=VinRobotics Motor Control Diagnostic
GenericName=Motor-control diagnostic suite
Comment=Bring up, tune, and diagnose CiA 402 / CAN-FD motor drives
Icon=vr_mc_diagnostic
Exec=vr_mc_diagnostic
Terminal=false
Categories=Development;Engineering;Robotics;Utility;
Keywords=motor;control;cia402;canopen;canfd;ethercat;tuning;vinrobotics;
StartupWMClass=vr_mc_diagnostic
EOF

# -- build the AppImage ---------------------------------------------------
# The qt plugin walks the binary's deps, copies + relocates all Qt6 libs
# + QPA platform plugins (xcb / wayland). --output appimage produces the
# final .AppImage in the cwd, so cd into dist/ first.
mkdir -p "${DIST}"
cd "${DIST}"

VERSION="${VERSION}" \
"${LDP_RUN[@]}" \
    --appdir       "${APPDIR}" \
    --plugin       qt \
    --output       appimage \
    --executable   "${APPDIR}/usr/bin/vrmc_sim" \
    --desktop-file "${APPDIR}/usr/share/applications/vr_mc_diagnostic.desktop" \
    --icon-file    "${APPDIR}/usr/share/icons/hicolor/256x256/apps/vr_mc_diagnostic.png"

# -- final touches --------------------------------------------------------
RAW=$(ls -t "${DIST}"/*.AppImage 2>/dev/null \
        | grep -v -- '--' | head -1 || true)
# (the linuxdeploy/Qt-plugin AppImages cached under tools/.cache have
#  '-x86_64' / '--' in the name; grep filters them out so we only see
#  the freshly-built output)
if [[ -n "${RAW}" ]]; then
    OUT="${DIST}/VinRobotics_MotorControl_Diagnostic_Tool-${VERSION}-${ARCH}.AppImage"
    mv "${RAW}" "${OUT}"
    chmod +x "${OUT}"
    echo
    echo "================================================================"
    echo "  Built: ${OUT}"
    echo "  Size:  $(du -h "${OUT}" | awk '{print $1}')"
    echo
    echo "  Contents:"
    echo "    /usr/bin/vr_mc_diagnostic    (GUI)"
    echo "    /usr/bin/vrmc_sim            (Help → Start demo spawns this)"
    echo "    /usr/lib/libcontrolcanfd.so  (ZLG USB-CANFD vendor lib)"
    echo "    /usr/share/vr_mc_diagnostic/docs/"
    echo "================================================================"
else
    echo "ERROR: linuxdeploy did not produce an AppImage" >&2
    exit 2
fi
