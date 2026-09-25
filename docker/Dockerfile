# 增量层:基于已编译好的基础镜像,只在其上叠加新内容。
ARG RM_BASE_IMAGE=rm_vision-base:ubuntu22.04
FROM ${RM_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# 新增 apt 依赖:在此追加
# 新增第三方源码编译:在此追加

WORKDIR /work
CMD ["bash"]
