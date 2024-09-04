#!/usr/bin/env bash

export BASE_IMAGE_NAME= ""
export REGISTRY=<your dockerhub username>

# build base image for Jetson Nano
DOCKER_BUILDKIT=1 docker build \
  --platform linux/arm64 \
  --network host\
  -f Dockerfile.aarch64 \
  -t ${REGISTRY}/${BASE_IMAGE_NAME}\
  --push .

# build ROS2
DOCKER_BUILDKIT=1 docker build \
  --platform linux/arm64 \
  --network host\
  --build-arg BASE_IMAGE="${BASE_IMAGE_NAME}"\
  -f Dockerfile.aarch64\
  -t ${REGISTRY}/jnano_ros2:latest\
  --push .