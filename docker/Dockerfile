FROM aerostack2/nightly-humble:latest 

# Install Gazebo Harmonic
RUN apt remove ignition* -y \
    && apt autoremove -y
RUN apt-get update \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y -q \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y -q \
    ros-humble-ros-gzharmonic \
    ros-humble-plotjuggler-ros \
    python-is-python3 \
    && rm -rf /var/lib/apt/lists/*

RUN echo "set -g mouse on" > ~/.tmux.conf 

WORKDIR /root/aerostack2_ws/src/aerostack2
ARG AS2_BRANCH
RUN git checkout $AS2_BRANCH
WORKDIR /root/aerostack2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Clone demo project
WORKDIR /root/
RUN git clone https://github.com/aerostack2/project_sherec_navigation.git sherec_nav

WORKDIR /root/sherec_nav