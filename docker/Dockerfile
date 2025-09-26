FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN useradd -rm -d /home/user_name -s /bin/bash -g root -G sudo -u 1001 user_name 

USER user_name

WORKDIR /home/user_name

RUN mkdir -p /home/user_name/aprs-ws/src/gz_pkgs

WORKDIR /home/user_name/aprs_ws

ENV GZ_VERSION=harmonic

RUN source /opt/ros/jazzy/setup.bash && \
    sudo apt-get update && \
    sudo apt-get install lsb-release gnupg python3-pip python3-setuptools wget ros-jazzy-ur-description ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-interfaces -y && \
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    sudo apt-get update -qq && \
    sudo apt-get -y install gz-harmonic && \
    git clone https://github.com/usnistgov/aprs_lab_sim.git src/aprs_lab_sim && \
    git clone https://github.com/usnistgov/aprs-ros-conveyor.git src/aprs_ros_conveyor -b main && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

RUN echo 'source /home/user_name/aprs_ws/install/setup.bash' >> /home/user_name/.bashrc
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/user_name/.bashrc
RUN echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/opt/ros/jazzy/share' >> /home/user_name/.bashrc