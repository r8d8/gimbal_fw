#!/usr/bin/env bash

export REGISTRY="ghcr.io/r8d8/jnano_comp"

docker login ghcr.io -u r8d8 -p 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

# # build base image for Jetson Nano
# DOCKER_BUILDKIT=1 docker build \
#   --platform linux/arm64 \
#   --network host\
#   -f Dockerfile.aarch64 \
#   -t ${REGISTRY}/${BASE_IMAGE_NAME}\
#   --push .

# build ROS2
DOCKER_BUILDKIT=1 docker build \
  --platform linux/arm64 \
  --network host\
  --progress=plain\
  -f Dockerfile.l4t_ros2humble\
  -t ${REGISTRY}/jnano_ros2:latest\
  --push .