#!/usr/bin/env bash
set -euo pipefail

OPENCV_VER=4.10.0
BUILD_JOBS=4
INSTALL_PREFIX=/usr/local
WORK_DIR="${HOME}/opencv-build"
REGISTER_LDCONFIG=0
USE_GITEE=0
GITEE_REPO="https://gitee.com/mirrors/opencv.git"

# --- pinned checksums (OpenCV 4.10.0) ---
SRC_MD5="adaf23e87339e6df6d50d68001138ccc"
SRC_SIZE=94993429
TAG_COMMIT="71d3237a093b60a27601c20e9ee6c3e52154e8b1"
IPPICV_COMMIT="fd27188235d85e552de31425e7ea0f53ba73ba53"
IPPICV_NAME="ippicv_2021.11.0_lnx_intel64_20240201_general.tgz"
IPPICV_MD5="0f2745ff705ecae31176dad437608f6f"
ADE_NAME="v0.1.2d.zip"
ADE_MD5="dbb095a8bf3008e91edbbf45d8d34885"

usage() {
  cat <<EOF
Usage: $0 [options]
  -j N                   parallel build jobs (default: 4)
  --prefix DIR           install prefix (default: /usr/local)
  --workdir DIR          source/build/cache dir (default: ~/opencv-build)
  --gitee                clone OpenCV source from gitee instead of tarball mirror
  --register-ldconfig    write /etc/ld.so.conf.d/opencv-4.10.conf and run ldconfig
  -h, --help             show this help

Env:
  OPENCV_MIRROR          force a single mirror prefix, e.g. https://gh.xxooo.cf/
                         (default: try all known mirrors, verifying md5 on each)
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    -j*) BUILD_JOBS="${1#-j}" ;;
    --jobs) BUILD_JOBS="$2"; shift ;;
    --prefix) INSTALL_PREFIX="$2"; shift ;;
    --workdir) WORK_DIR="$2"; shift ;;
    --gitee) USE_GITEE=1 ;;
    --register-ldconfig) REGISTER_LDCONFIG=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

ok()   { printf '  \033[32mOK\033[0m   %s\n' "$*"; }
warn() { printf '  \033[33mWARN\033[0m %s\n' "$*"; }
die()  { printf 'error: %s\n' "$*" >&2; exit 1; }

md5_of() { md5sum "$1" | cut -d' ' -f1; }

verify_md5() {
  local file="$1" want="$2" label="$3" got
  [ -f "$file" ] || return 1
  got="$(md5_of "$file")"
  if [ "$got" = "$want" ]; then
    ok "$label md5 $got"
    return 0
  fi
  warn "$label md5 mismatch: got $got want $want"
  return 1
}

detect_distro() {
  local id like
  id="$(. /etc/os-release && echo "${ID:-}")"
  like="$(. /etc/os-release && echo "${ID_LIKE:-}")"
  case "${id} ${like}" in
    *fedora*|*rhel*) echo fedora ;;
    *debian*|*ubuntu*) echo debian ;;
    *) echo "" ;;
  esac
}

install_deps() {
  case "$1" in
    fedora)
      sudo dnf install -y cmake ninja-build gcc-c++ pkgconf-pkg-config \
        tbb-devel libv4l-devel gtk3-devel \
        gstreamer1-devel gstreamer1-plugins-base-devel \
        curl tar unzip bzip2
      ;;
    debian)
      sudo apt-get update
      sudo apt-get install -y --no-install-recommends \
        cmake ninja-build g++ pkg-config \
        libtbb-dev libv4l-dev libgtk-3-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        curl tar unzip bzip2 ca-certificates
      ;;
  esac
}

if [ -n "${OPENCV_MIRROR+x}" ]; then
  MIRRORS=("$OPENCV_MIRROR")
else
  MIRRORS=("https://gh.xxooo.cf/" "https://gh-proxy.com/" "https://ghproxy.net/" "")
fi

# fetch <url-without-mirror> <dest> <md5> <label>
# tries each mirror in turn; keeps only a file whose md5 matches
fetch() {
  local url="$1" dest="$2" want="$3" label="$4" prefix tmp
  for prefix in "${MIRRORS[@]}"; do
    [ -n "$prefix" ] && echo "  ($label) trying $prefix" || echo "  ($label) trying github.com direct"
    tmp="${dest}.part"
    rm -f "$tmp"
    if curl -fL --retry 2 --retry-delay 2 -C - -o "$tmp" "${prefix}${url}" 2>/dev/null; then
      if verify_md5 "$tmp" "$want" "$label"; then
        mv -f "$tmp" "$dest"
        return 0
      fi
    else
      warn "$label download failed from $prefix"
    fi
    rm -f "$tmp"
  done
  return 1
}

DISTRO="$(detect_distro)"
[ -n "$DISTRO" ] || die "unsupported distro (need Fedora/RHEL or Debian/Ubuntu)"
echo "==> distro: $DISTRO"
echo "==> installing dependencies"
install_deps "$DISTRO"

mkdir -p "$WORK_DIR"
cd "$WORK_DIR"
CACHE="$WORK_DIR/.cache"
SRC_DIR="opencv-${OPENCV_VER}"

# ---- 1. source ----
if [ ! -d "$SRC_DIR" ]; then
  if [ "$USE_GITEE" -eq 1 ]; then
    echo "==> cloning OpenCV ${OPENCV_VER} from gitee"
    git clone --depth 1 -b "${OPENCV_VER}" "$GITEE_REPO" "$SRC_DIR"
    got="$(git -C "$SRC_DIR" rev-parse HEAD)"
    [ "$got" = "$TAG_COMMIT" ] || die "gitee clone commit mismatch: $got != $TAG_COMMIT"
    ok "gitee clone commit $got"
  else
    echo "==> fetching OpenCV ${OPENCV_VER} source"
    SRC_TAR="$WORK_DIR/opencv-${OPENCV_VER}.tar.gz"
    if ! verify_md5 "$SRC_TAR" "$SRC_MD5" "source tarball"; then
      rm -f "$SRC_TAR"
      fetch "https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VER}.tar.gz" \
        "$SRC_TAR" "$SRC_MD5" "source tarball" || die "could not fetch a valid source tarball"
    fi
    [ "$(stat -c%s "$SRC_TAR")" = "$SRC_SIZE" ] || die "source tarball size mismatch"
    tar xzf "$SRC_TAR"
  fi
fi

# ---- 2. third-party deps (pre-seed cmake cache with verified copies) ----
echo "==> fetching third-party deps"
mkdir -p "$CACHE/ippicv" "$CACHE/ade"
IPP_DEST="$CACHE/ippicv/${IPPICV_MD5}-${IPPICV_NAME}"
ADE_DEST="$CACHE/ade/${ADE_MD5}-${ADE_NAME}"
if ! verify_md5 "$IPP_DEST" "$IPPICV_MD5" "ippicv"; then
  rm -f "$IPP_DEST"
  fetch "https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_COMMIT}/ippicv/${IPPICV_NAME}" \
    "$IPP_DEST" "$IPPICV_MD5" "ippicv" || die "could not fetch valid ippicv"
fi
if ! verify_md5 "$ADE_DEST" "$ADE_MD5" "ade"; then
  rm -f "$ADE_DEST"
  fetch "https://github.com/opencv/ade/archive/${ADE_NAME}" \
    "$ADE_DEST" "$ADE_MD5" "ade" || die "could not fetch valid ade"
fi

# ippicv/ADE are pre-verified above; OPENCV_DOWNLOAD_PATH makes cmake reuse them
# (cmake checks the md5 itself again, so a tampered cache can never be used)
#
# cmake >= 4 removes compatibility with cmake_minimum_required(<3.5). OpenCV
# hardware_check/OpenCVGenPkgconfig.cmake still ship such calls, and the latter
# runs as a separate `cmake -P` process that does NOT inherit -D cache vars,
# so export it. The variable only exists since cmake 4.0 -- on Ubuntu 22.04
# (cmake 3.22) and Debian 13 (cmake 3.31) it is silently ignored, no-op.
export CMAKE_POLICY_VERSION_MINIMUM=3.5
cmake -S "$SRC_DIR" -B build -G Ninja \
  -D CMAKE_BUILD_TYPE=Release \
  -D CMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
  -D WITH_CUDA=OFF -D WITH_OPENCL=OFF -D WITH_FFMPEG=OFF \
  -D WITH_IPP=ON -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_GSTREAMER=ON \
  -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF \
  -D BUILD_opencv_python3=OFF -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D OPENCV_DOWNLOAD_PATH="$CACHE" \
  -D OPENCV_IPPICV_URL="https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_COMMIT}/ippicv/" \
  -D OPENCV_ADE_URL="https://github.com/opencv/ade/archive/"

echo "==> building (jobs=$BUILD_JOBS)"
cmake --build build -j"$BUILD_JOBS"

echo "==> installing to $INSTALL_PREFIX"
sudo cmake --install build

if [ "$REGISTER_LDCONFIG" -eq 1 ]; then
  LIBDIR="$INSTALL_PREFIX/lib64"
  [ -d "$LIBDIR" ] || LIBDIR="$INSTALL_PREFIX/lib"
  echo "$LIBDIR" | sudo tee /etc/ld.so.conf.d/opencv-4.10.conf >/dev/null
  sudo ldconfig
  ok "registered $LIBDIR in /etc/ld.so.conf.d/opencv-4.10.conf"
fi

cat <<EOF
==> done: OpenCV ${OPENCV_VER} installed to $INSTALL_PREFIX
    CMake:  find_package(OpenCV 4.10 REQUIRED)
    RPATH:  add "$INSTALL_PREFIX/lib64" to your target (unless --register-ldconfig)
    Verify: pkg-config --modversion opencv4
EOF
