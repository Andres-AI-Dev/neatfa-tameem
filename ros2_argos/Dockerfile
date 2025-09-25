# ROS2 Humble + ARGoS3 + PPO Environment for macOS Docker
FROM ubuntu:22.04

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    wget \
    git \
    cmake \
    build-essential \
    python3-pip \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install ARGoS3 dependencies
RUN apt-get update && apt-get install -y \
    libfreeimage-dev \
    libfreeimageplus-dev \
    qtbase5-dev \
    qt5-qmake \
    libqt5opengl5-dev \
    freeglut3-dev \
    libxi-dev \
    libxmu-dev \
    liblua5.3-dev \
    lua5.3 \
    libxml2-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install ARGoS3 from source
WORKDIR /root
RUN git clone https://github.com/ilpincy/argos3.git \
    && cd argos3 \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release \
            -DARGOS_BUILD_FOR=simulator \
            -DARGOS_THREADSAFE_LOG=ON \
            -DARGOS_DYNAMIC_LOADING=ON \
            -DARGOS_DOCUMENTATION=OFF ../src \
    && make -j$(nproc) \
    && make install \
    && ldconfig

# Install Python ML dependencies
RUN pip3 install --upgrade pip \
    && rm -rf /usr/lib/python3/dist-packages/sympy* \
    && rm -rf /usr/lib/python3/dist-packages/matplotlib* \
    && rm -rf /usr/lib/python3/dist-packages/numpy* \
    && rm -rf /usr/lib/python3/dist-packages/opencv* \
    && rm -rf /usr/lib/python3/dist-packages/cv2* \
    && pip3 install \
        'numpy<2.0.0' \
        stable-baselines3 \
        gymnasium \
        torch \
        tensorboard \
        matplotlib \
        opencv-python

# Create ROS2 workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Clone ARGoS-ROS2 bridge repositories
RUN cd src \
    && git clone https://github.com/CPS-Konstanz/argos3-ros2-bridge.git \
    && git clone https://github.com/BOTSlab/argos_bridge.git argos_bridge_reference

# Source ROS2 setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Build the ROS2 workspace (initial build without plugins)
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash \
    && cd /root/ros2_ws/src/argos3-ros2-bridge \
    && sed -i 's/add_subdirectory(plugins)/#add_subdirectory(plugins)/' CMakeLists.txt \
    && cd /root/ros2_ws \
    && colcon build --packages-select argos3_ros2_bridge

# Build with plugins enabled
RUN source /opt/ros/humble/setup.bash \
    && cd /root/ros2_ws/src/argos3-ros2-bridge \
    && sed -i 's/#add_subdirectory(plugins)/add_subdirectory(plugins)/' CMakeLists.txt \
    && cd /root/ros2_ws \
    && colcon build --packages-select argos3_ros2_bridge

# Set ARGoS environment variables
ENV ARGOS_PLUGIN_PATH=/root/ros2_ws/install/argos3_ros2_bridge/lib
ENV LD_LIBRARY_PATH=/usr/local/lib/argos3:${LD_LIBRARY_PATH:-}

# Create directory for our PPO implementation
RUN mkdir -p /root/ppo_argos
WORKDIR /root/ppo_argos

# Entry point
CMD ["/bin/bash"]