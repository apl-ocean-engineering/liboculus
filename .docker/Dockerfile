ARG UBUNTU_VERSION=focal

#====================================================================
# Preliminary image with dependencies
#====================================================================
#
## This should work on both amd64 and arm64 through the magic of multi-arch?
FROM ubuntu:${UBUNTU_VERSION} AS deps

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    cmake \
    gcc \
    git \
    libboost1.71-all-dev \
    python3 \
    python3-apt \
    python3-distutils \
    python3-pip \
    python-is-python3 \
    && apt autoremove -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

#RUN pip3 install distutils

#====================================================================
# Intermediate image with prereq steps for running in CI
#====================================================================
FROM deps as ci

## ~~ Create non-root user ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ARG USERNAME=apl
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  #
  # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/apl

CMD ["/bin/bash"]

#====================================================================
# Intermediate image with cloned Git repos
#====================================================================
FROM ci as git_checkout

ARG WS_DIR=/home/$USERNAME/ros_ws
ONBUILD WORKDIR ${WS_DIR}/src

ARG LIBOCULUS_REPO=https://github.com/apl-ocean-engineering/liboculus.git
ARG LIBOCULUS_BRANCH=main
ONBUILD RUN echo "Cloning from ${LIBOCULUS_BRANCH} branch ${LIBOCULUS_ROS_REPO}"

ONBUILD RUN git clone --depth 1 -b ${LIBOCULUS_BRANCH} ${LIBOCULUS_REPO}

#====================================================================
# Final image with built ROS packages
#====================================================================
# dockerfile_lint - ignore
FROM git_checkout as liboculus

ARG WS_DIR
LABEL Version=0.1
LABEL Name=arena_camera_ros

WORKDIR ${WS_DIR}/liboculus
RUN ./fips build
