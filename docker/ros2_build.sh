#!/usr/bin/env bash

# define a registry to push the images to
#export REGISTRY=<your dockerhub username>
# create new buildx that support multiple platforms
docker buildx create --use  --driver-opt network=host --name MultiPlatform

# build the image for two different platforms and push the images
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f ros2_base.Dockerfile \
  --push .