ARG BASE_OS=nvcr.io/nvidia/cuda:12.5.1-base-ubuntu24.04

FROM ${BASE_OS}

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y full-upgrade

# Configuración básica
RUN apt-get update && apt-get install -y \
    locales sudo \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Dependencias principales
RUN apt-get update && apt-get install -y \
    git wget curl vim python3-pip \
    tigervnc-standalone-server \
    openssh-server \
    dbus-x11 x11-xserver-utils xauth \
    xfce4 xfce4-goodies \
    mesa-utils libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# ROS2
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y ros-jazzy-desktop \
    && rm -rf /var/lib/apt/lists/*

# Herramientas de desarrollo ROS2
RUN apt-get update && apt-get install -y \
    python3-flake8-blind-except \
    python3-flake8-class-newline \
    python3-flake8-deprecated \
    python3-mypy \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-mock \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-runner \
    python3-pytest-timeout \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Gazebo
RUN apt-get update && apt-get install -y ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# PyTorch y MuJoCo
RUN pip3 install torch torchvision torchaudio mujoco --break-system-packages

# Configuración de VNC
RUN mkdir -p /root/.vnc && \
    echo "password" | vncpasswd -f > /root/.vnc/passwd && \
    chmod 600 /root/.vnc/passwd

RUN echo '#!/bin/sh\n\nunset SESSION_MANAGER\nunset DBUS_SESSION_BUS_ADDRESS\n\nexport XDG_RUNTIME_DIR=/tmp/xdg-runtime\nmkdir -p $XDG_RUNTIME_DIR\nchmod 700 $XDG_RUNTIME_DIR\n\nexport XKL_XMODMAP_DISABLE=1\nexec startxfce4\n' > /root/.vnc/xstartup && \
    chmod +x /root/.vnc/xstartup

# Configuración SSH
RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    echo "root:docker" | chpasswd

EXPOSE 22 5901

ENTRYPOINT ["/bin/bash", "-c", "service ssh restart && rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 && vncserver :1 -geometry 1920x1080 -depth 24 -localhost no && tail -f /root/.vnc/*.log"]