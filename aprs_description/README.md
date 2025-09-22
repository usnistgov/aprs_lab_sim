# APRS_description
Loads the robots from the APRS lab in Gazebo Harmonic Running Iron

## Packages
* `aprs_description` - Includes all descriptions for the four robots.

## Installation Instructions
* Pull the docker image

    `docker image pull jaybrecht/aprs-description`

* Create a container using the docker image

    `docker run -t -d --name aprs-description -e DISPLAY=$DISPLAY -e LOCAL_USER_ID=1001  --gpus=all --runtime=nvidia -e "NVIDIA_DRIVER_CAPABILITIES=all" --network=host --pid=host --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:rw jaybrecht/aprs-description:latest`

* Enable local connections to docker

    `xhost +local:docker`

* Access the docker container through the terminal

    `docker exec -it aprs-description bash`

* Clone the description package

    `git clone https://github.com/usnistgov/aprs_description.git src/aprs_description`

* Clone the plugins package

    `git clone https://github.com/jfernandez37/aprs_plugins.git src/aprs_plugins`

* Clone the conveyor package

    `git clone https://github.com/usnistgov/aprs-ros-conveyor.git src/aprs_ros_conveyor`

* Remove the old gz_ros2_control package

    `rm -rf src/gz_ros2_control`

* Clone the new gz_ros2_control package in the iron branch

    `git clone https://github.com/jfernandez37/gz_ros2_control.git src/gz_ros2_control -b iron`

* Set the resource path

    `export GZ_SIM_RESOURCE_PATH=/home/ubuntu/aprs_ws/install/ur_description/share/:/home/ubuntu/aprs_ws/install/aprs_description/share/aprs_description/gz_models/:/home/ubuntu/aprs_ws/install/aprs_description/share`

* Build the workspace

  `colcon build`

* Source the workspace

    `. install/setup.bash`

* Launch the environment with the robots in the same namespace

    `ros2 launch aprs_description load_aprs_robots.launch.py`
  
* Launch the environment with the robots in different namespaces

    `ros2 launch aprs_description load_aprs_robots_ns.launch.py`
