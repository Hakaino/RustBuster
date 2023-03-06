# This is an auto generated Dockerfile for ros:perception
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-base-jammy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-perception=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get install ros-humble-joint-state-publisher-gui ros-humble-tf-transformations ros-humble-xacro
RUN apt-get install ros-humble-depth-image-proc
RUN apt-get install ros-humble-librealsense2
RUN pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core

# install boston dynamics stuff through deb
RUN wget -q -O /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb \
    https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/v0.0.0-humble/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
RUN dpkg -i /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
RUN rm /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
RUN -q -O /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb \
    https://github.com/bdaiinstitute/spot_ros2/releases/download/spot_msgs-v0.0-0/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb
RUN dpkg -i /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb
RUN rm /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb


# rosdep init -- do i need this?
RUN rosdep init
RUN rosdep update


# Clone packages
WORKDIR /root/workspace/src
RUN git clone https://github.com/Hakaino/RustBuster.git
RUN git clone https://github.com/bdaiinstitute/spot_ros2.git
RUN git clone https://github.com/MASKOR/Spot-ROS2.git
RUN git clone https://github.com/robo-friends/m-explore-ros2.git

# Install dependencies
WORKDIR /root/workspace/
RUN rosdep install -i --from-path src --rosdistro humble -y
RUN colcon build --symlink-install