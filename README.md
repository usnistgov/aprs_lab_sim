# APRS_ros2_demo

# Installation

1. For an easy installation, use the docker container with all dependencies and libraries already installed:

    `docker pull jfernandez37/aprs-ros2-demo`

2. Allow for the terminal you are in to load GUI's on the host device:

    `xhost +local:docker`

3. Start a new container using the docker image:

    `docker run -t -d --name aprs-ros2-demo -e DISPLAY=$DISPLAY -e LOCAL_USER_ID=1001  --gpus=all --runtime=nvidia -e "NVIDIA_DRIVER_CAPABILITIES=all" --network=host --pid=host --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:rw jfernandez37/aprs-ros2-demo`

4. Open the container in the terminal:

    `docker exec -it aprs-ros2-demo bash`

5. Source all of the correct resources:

    `echo "export GZ_SIM_RESOURCE_PATH=/home/ubuntu/gz_ws/install/ur_description/share/:/home/ubuntu/aprs_ws/install/aprs_gz_sim/share/aprs_gz_sim/gz_models/:/home/ubuntu/aprs_ws/install/aprs_description/share:/home/ubuntu/aprs_ws/install/aprs_gz_sim/share:/home/ubuntu/aprs_ws/install/aprs_gz_sim/share/aprs_gz_sim/worlds:/home/ubuntu/aprs_ws/install/aprs_gz_sim/share/aprs_gz_sim/models:/home/ubuntu/aprs_ws/install/ariac_gz_plugins/share/ariac_gz_plugins/models" > ~/.bashrc`

6. To open the simulation, use this command:

    `ros2 launch aprs_gz_sim environment.launch.py use_seperate_descriptions:=True`

7. If you would like to test the the movement of the robots, use this command:

    `ros2 launch moveit_test individual_testing.launch.py`

# Run into problems?

It is possible that the docker image is not updated to the most recent version of the simulation. To solve this issue, run these commands

1. `cd ~/aprs_ws/src/APRS_ros2_demo_sim`
2. `git pull`
3. `git checkout new_demo`
4. `cd ~/aprs_ws`
5. `colcon build`
