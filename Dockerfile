FROM ubuntu:20.04

# Set locale
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt update && apt install -y curl gnupg2 lsb-release

# Install ROS1 and ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt-get update && apk add git
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-foxy-desktop-full ros-noetic-desktop-full
RUN apt-get install python3-rosdep -y
RUN apt-get install python3-colcon-common-extensions -y
RUN apt-get install ros-foxy-joint-state-publisher-gui ros-foxy-tf-transformations ros-foxy-xacro
RUN apt-get install ros-foxy-depth-image-proc
RUN apt-get install ros-foxy-librealsense2
RUN pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core




RUN wget -q -O /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/v0.0.0-humble/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
RUN dpkg -i /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
RUN rm /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
RUN -q -O /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb https://github.com/bdaiinstitute/spot_ros2/releases/download/spot_msgs-v0.0-0/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb
RUN dpkg -i /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb
RUN rm /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb

RUN rosdep init
RUN rosdep update


# Clone packages
WORKDIR /root/workspace/ros2/src
RUN git clone https://github.com/Hakaino/RustBuster.git
RUN git clone https://github.com/bdaiinstitute/spot_ros2.git
RUN git clone https://github.com/MASKOR/Spot-ROS2.git
RUN git clone https://github.com/robo-friends/m-explore-ros2.git
WORKDIR /root/workspace/ros1/src

# Install dependencies
WORKDIR /root/workspace/ros2/
RUN rosdep install -i --from-path src --rosdistro foxy -y
RUN colcon build --symlink-install

WORKDIR /root/workspace/ros1/
RUN rosdep install -i --from-path src --rosdistro noetic -y
RUN colcon build --symlink-install
