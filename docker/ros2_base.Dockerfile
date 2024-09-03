# https://www.docker.com/blog/faster-multi-platform-builds-dockerfile-cross-compilation-guide/

FROM nvcr.io/nvidia/cudagl:11.4.2-devel-ubuntu18.04 as amd64_base
FROM nvcr.io/nvidia/l4t-base:r32.7.0 as arm64_base
FROM ${TARGETARCH}_base as dev

SHELL ["/bin/bash", "-c"] 
ENV SKIP_ROSDEP=""

# https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros2.foxy
# https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html

ENV ROS_DISTRO=jazzy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3
ENV SHELL /bin/bash

ENV DEBIAN_FRONTEND=noninteractive
ENV RTI_NC_LICENSE_ACCEPTED=yes

# change the locale from POSIX to UTF-8
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt update \
  && apt upgrade -y \
  && apt install -y \
    software-properties-common \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    terminator \
    build-essential \
    cmake \
    git \
    python3 \
    libpython3-dev \
    xorg-dev \
    libusb-1.0-0-dev \
    libxinerama-dev \
    python3 \
    python3-dev \
    libpython3.8-dev \
    gcc-8 g++-8 \
  && rm /usr/bin/gcc /usr/bin/g++ \
  && ln -s gcc-8 /usr/bin/gcc \
  && ln -s g++-8 /usr/bin/g++ \
  && add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

RUN apt update && apt install -y \
  libbullet-dev \
  python3-pip \
  python3-pytest-cov \
  # install some pip packages needed for testing
  && python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    # install Fast-RTPS dependencies
  && apt install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev \
    # install Cyclone DDS dependencies
    libcunit1-dev
  
RUN apt install -y \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-rosdep \
  python3-vcstool \
  python3-rosinstall-generator \
  && python -m pip install pip install setuptools==58.2.0


RUN python3 -m pip install --upgrade pip && python3 -m pip install --upgrade --no-cache-dir --verbose cmake
RUN cmake --version


# get ROS2 code
RUN mkdir -p ${ROS_ROOT}/src \
  && cd ${ROS_ROOT} \
  && vcs import --input https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos src

RUN echo 'export ROS_PACKAGE_PATH="${ROS_ROOT}/src"' > /setup_ROS_PACKAGE_PATH.sh \
  && echo 'for dir in ${ROS_ROOT}/src/*; do export ROS_PACKAGE_PATH="$dir:$ROS_PACKAGE_PATH"; done' >> /setup_ROS_PACKAGE_PATH.sh \
  && echo "source /setup_ROS_PACKAGE_PATH.sh >> /etc/bash.bashrc"

ENV ROSDEP_SKIP_PACKAGES="fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
RUN cd ${ROS_ROOT} && source /setup_ROS_PACKAGE_PATH.sh \
  && apt upgrade -y \
  && rosdep init \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -y --skip-keys="$ROSDEP_SKIP_PACKAGES"

RUN cd ${ROS_ROOT} \
  && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN . ${ROS_ROOT}/install/local_setup.bash \
  && echo "source $ROS_ROOT/install/setup.bash" >> /etc/bash.bashrc \
  && echo "source $ROS_ROOT/install/local_setup.bash" >> /etc/bash.bashrc

RUN TEST_PLUGINLIB_PACKAGE="${ROS_ROOT}/build/pluginlib/pluginlib_enable_plugin_testing/install/test_pluginlib__test_pluginlib/share/test_pluginlib/package.xml" && \
  sed -i '/<\/description>/a <license>BSD<\/license>' $TEST_PLUGINLIB_PACKAGE && \
  sed -i '/<\/description>/a <maintainer email="michael@openrobotics.org">Michael Carroll<\/maintainer>' $TEST_PLUGINLIB_PACKAGE && \
  cat $TEST_PLUGINLIB_PACKAGE

RUN source $ROS_ROOT/install/setup.bash \
  && rosdep install --from-paths src --ignore-src -r -y --rosdistro=$ROS_DISTRO --skip-keys="$ROSDEP_SKIP_PACKAGES" \
  && colcon build --merge-install --install-base "$ROS_ROOT/install" --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to-regex pcl_ros nav2* rtabmap*


# Manually install MAVROS from source in the ws_blue/ workspace
WORKDIR /tmp/src/
ARG MAVROS_RELEASE=ros2
ARG MAVLINK_RELEASE=release/rolling/mavlink
RUN git clone --depth 1 -b ${MAVROS_RELEASE} https://github.com/mavlink/mavros.git
RUN git clone --depth 1 --recursive -b ${MAVLINK_RELEASE} https://github.com/mavlink/mavlink-gbp-release.git mavlink

# mavgen uses future.standard_library for backwards compatibility with Python2;
# However, this caused issues in Python3.12.
COPY mavlink_standard_library.patch .
RUN patch -d mavlink -p 1 < mavlink_standard_library.patch

WORKDIR /tmp
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --as-root=pip:false --rosdistro ${ROS_DISTRO} \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Install geographiclib datasets
# Scripts uses geographiclib-tools which is a rosdep dependency of mavros installed above
WORKDIR /tmp
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x /tmp/install_geographiclib_datasets.sh \
    && sudo /tmp/install_geographiclib_datasets.sh

RUN . ${ROS_ROOT}/install/local_setup.bash \
    && echo "source $ROS_ROOT/install/setup.bash" >> /etc/bash.bashrc \
    && echo "source $ROS_ROOT/install/local_setup.bash" >> /etc/bash.bashrc

RUN mkdir -p /root/ros2_ws/src && mkdir /root/ros2_ws_tutorial
WORKDIR /root/ros2_ws