ARG BASE_IMAGE=ubuntu:22.04
# MVS SDK 压缩包(必需):默认取构建上下文里的 MVS.tar.gz,可用 --build-arg MVS_TARBALL=<路径或URL> 覆盖。
ARG MVS_TARBALL=MVS.tar.gz
FROM ${BASE_IMAGE} AS base

ENV DEBIAN_FRONTEND=noninteractive

# apt 源换中科大(USTC)
RUN for f in /etc/apt/sources.list /etc/apt/sources.list.d/*.sources /etc/apt/sources.list.d/*.list; do \
      [ -f "$f" ] || continue; \
      sed -i \
        -e 's|http://archive.ubuntu.com/ubuntu|http://mirrors.ustc.edu.cn/ubuntu|g' \
        -e 's|http://security.ubuntu.com/ubuntu|http://mirrors.ustc.edu.cn/ubuntu|g' \
        -e 's|http://deb.debian.org/debian|http://mirrors.ustc.edu.cn/debian|g' \
        -e 's|http://security.debian.org/debian-security|http://mirrors.ustc.edu.cn/debian-security|g' \
        "$f"; \
    done

# 工具链 + 项目依赖。
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      ninja-build \
      git \
      pkg-config \
      libyaml-cpp-dev \
      libeigen3-dev \
      ca-certificates \
      sudo \
 && rm -rf /var/lib/apt/lists/*


# OpenCV 4.10 源码编译并安装
FROM base AS opencv410
ARG OPENCV_VER=4.10.0
RUN set -eu; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      libtbb-dev libv4l-dev libgtk-3-dev \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      curl unzip; \
    rm -rf /var/lib/apt/lists/*; \
    work=/tmp/opencv; cache=$work/.cache; \
    mkdir -p "$cache/ippicv" "$cache/ade"; cd "$work"; \
    fetch() { \
      url="$1"; dest="$2"; want="$3"; \
      for p in https://gh.xxooo.cf/ https://gh-proxy.com/ https://ghproxy.net/ ""; do \
        if curl -fL --retry 2 -o "$dest.part" "${p}${url}"; then \
          if [ "$(md5sum "$dest.part" | cut -d' ' -f1)" = "$want" ]; then mv "$dest.part" "$dest"; return 0; fi; \
        fi; \
        rm -f "$dest.part"; \
      done; \
      return 1; \
    }; \
    fetch "https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VER}.tar.gz" \
      "$work/opencv.tar.gz" "adaf23e87339e6df6d50d68001138ccc"; \
    fetch "https://raw.githubusercontent.com/opencv/opencv_3rdparty/fd27188235d85e552de31425e7ea0f53ba73ba53/ippicv/ippicv_2021.11.0_lnx_intel64_20240201_general.tgz" \
      "$cache/ippicv/0f2745ff705ecae31176dad437608f6f-ippicv_2021.11.0_lnx_intel64_20240201_general.tgz" "0f2745ff705ecae31176dad437608f6f"; \
    fetch "https://github.com/opencv/ade/archive/v0.1.2d.zip" \
      "$cache/ade/dbb095a8bf3008e91edbbf45d8d34885-v0.1.2d.zip" "dbb095a8bf3008e91edbbf45d8d34885"; \
    tar xzf "$work/opencv.tar.gz" -C "$work"; \
    export CMAKE_POLICY_VERSION_MINIMUM=3.5; \
    cmake -S "$work/opencv-${OPENCV_VER}" -B "$work/build" -G Ninja \
      -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D WITH_CUDA=OFF -D WITH_OPENCL=OFF -D WITH_FFMPEG=OFF \
      -D WITH_IPP=ON -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_GSTREAMER=ON \
      -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF \
      -D BUILD_opencv_python3=OFF -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D OPENCV_DOWNLOAD_PATH="$cache" \
      -D OPENCV_IPPICV_URL="https://raw.githubusercontent.com/opencv/opencv_3rdparty/fd27188235d85e552de31425e7ea0f53ba73ba53/ippicv/" \
      -D OPENCV_ADE_URL="https://github.com/opencv/ade/archive/"; \
    cmake --build "$work/build" -j4; \
    cmake --install "$work/build"; \
    libdir=/usr/local/lib; if [ -d /usr/local/lib64 ]; then libdir=/usr/local/lib64; fi; \
    echo "$libdir" > /etc/ld.so.conf.d/opencv-4.10.conf; ldconfig; \
    rm -rf "$work"


# 视频调试依赖
FROM opencv410 AS video-deps
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libavcodec-dev \
      libavutil-dev \
      libva-dev \
      mesa-va-drivers \
 && rm -rf /var/lib/apt/lists/*


# 海康 MVS SDK(相机驱动必需):解出到 /opt/MVS(CMake 默认 MVCAM_SDK_PATH)。
FROM video-deps AS mvs
ARG MVS_TARBALL
ADD ${MVS_TARBALL} /tmp/mvs-sdk.tar.gz
RUN mkdir -p /tmp/mvs /opt/MVS \
 && tar xzf /tmp/mvs-sdk.tar.gz -C /tmp/mvs \
 && tar xzf /tmp/mvs/MVS-*/MVS.tar.gz -C /opt/MVS \
 && rm -rf /tmp/mvs /tmp/mvs-sdk.tar.gz \
 && echo "/opt/MVS/lib/64" > /etc/ld.so.conf.d/mvs.conf \
 && ldconfig


# Rerun C++ SDK:从官方包编译安装到 /usr/local
# 版本 = ARG 里的固定 URL,升级时改这里。
FROM mvs AS rerun-sdk
ARG RERUN_SDK_URL=https://github.com/rerun-io/rerun/releases/download/0.38.1/rerun_cpp_sdk.zip
RUN apt-get update \
 && apt-get install -y --no-install-recommends curl unzip ca-certificates \
 && rm -rf /var/lib/apt/lists/* \
 && curl -fsSL "${RERUN_SDK_URL}" -o /tmp/rerun_cpp_sdk.zip \
 && unzip -q /tmp/rerun_cpp_sdk.zip -d /tmp/rerun_cpp_sdk \
 && cmake -S /tmp/rerun_cpp_sdk -B /tmp/rerun-sdk-build -G Ninja \
      -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
 && cmake --build /tmp/rerun-sdk-build -j4 \
 && cmake --install /tmp/rerun-sdk-build \
 && rm -rf /tmp/rerun_cpp_sdk.zip /tmp/rerun_cpp_sdk /tmp/rerun-sdk-build


# 项目本体:编译并安装
FROM rerun-sdk AS rerun
COPY . /tmp/rm-src
RUN cmake -S /tmp/rm-src -B /tmp/rm-build -G Ninja \
      -D CMAKE_BUILD_TYPE=Release \
      -D RM_DEBUG=ON \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
 && cmake --build /tmp/rm-build -j4 \
 && cmake --install /tmp/rm-build \
 && rm -rf /tmp/rm-src /tmp/rm-build


# 镜像终态:开发时把仓库挂载到 /work 使用。
FROM rerun AS final
WORKDIR /work
CMD ["bash"]
