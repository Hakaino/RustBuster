# RustBuster

This project uses **ROS2** (Humble) to perform **Autonomous Simultaneous Localization and Mapping** (A-SLAM) using
a **Spot** robot from Boston Dynamics.

<img src="dog.png" alt="dog" height="300"/>

<!---## Motivation
Offshore oil platforms are critical infrastructure that must be maintained to ensure their safety and longevity. Regular inspections are required to identify and address any signs of corrosion or other forms of degradation. However, these inspections can be dangerous and time-consuming for human inspectors, making it difficult to perform regular, thorough checks.
RustBuster was created to address this challenge by providing a safe, efficient, and automated solution for inspecting offshore oil platforms. The Spot robot is able to access difficult-to-reach areas and collect data on the condition of the platform, making it possible to perform regular and comprehensive inspections.
to inspect offshore oil platforms for rust and other forms of corrosion.--->

## System Hardware

* Spot robot
* RealSense Intel D455 (RGB-D and IMU)
* Laptop
* Ethernet cable (optional)

The intel d455 is used instead of spot's, native cameras because it has a higher frame rate and IMU.\
**NOTE:** It is possible to calculate Spot's IMU using the odometry messages, but the method is slow and imprecise.

## Installation

This system was developed and tested only in a Lenovo IdeaPad Gaming 3 15ARH05 laptop running **Ubuntu 22.04**.

Install dependencies from public repositories:

   ```
   sudo apt update
   
   # Boston Dynamics software
   pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
   
   # Ros related packages
   sudo apt install -y \
   ros-humble-nav2* \
   ros-humble-imu-filter-madgwick* \
   ros-humble-librealsense2* \
   ros-humble-apriltag* \
   ros-humble-usb-cam*     # (Optional)
   ```
> :warning: If RTAB-Map does not install correctly, download it from source and follow the installation instructions.
Clone repositories into ROS2 workspace:

   ```
   mkdir -p ros2_ws/src && cd ros2_ws/src 
   git clone https://github.com/Hakaino/RustBuster
   git clone https://github.com/bdaiinstitute/spot_ros2.git
   git clone https://github.com/robo-friends/m-explore-ros2
   cd ..
   rosdep install --from-paths src --ignore-src -r -y 
   ```

## Setup Spot

<img src="spot.png" alt="drawing" width="300"/>

Follow the
tutorial [Spot network setup](https://support.bostondynamics.com/s/article/Spot-network-setup#ConnecttoSpotviaDirectEthernet).

<details>
<summary>AAU students</summary>
I developed this project as a student at Aalborg University (AAU). \
To other students working with spot, I suggest:

* Contact [Frank](https://vbn.aau.dk/da/persons/frank-rasmussen) to get access to Spot.
* From the administrator account in Spot's computer, define a new user account.

</details>

## Run the program

1. Connect Spot and D455 to laptop. (I placed the laptop on back of the robot)
2. Start Spot and connect the controller.
   ([instructions](https://support.bostondynamics.com/s/article/Startup-Procedure#StartupProcedure))
3. Configure the fields **username**, **password**, **hostname** in [spot_config.yaml](config/spot_config.yaml)

> :warning: If **auto_stand** is set True, the robot will stand up

4. In the terminal, navigate to **ROS2_WS** folder generated during installation, and run the command:
   ```
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch rustbuster rustbuster_launch.py
   ```
## Test videos
[Manual Navigation](https://www.youtube.com/watch?v=ALY6QVHlzWc&t=1s): Using spot's controller\
[Semi-auto Navigation](https://www.youtube.com/watch?v=4guVVQsY4lg): Inputting navigation goals \
[Auto Navigation](https://www.youtube.com/watch?v=bm_8iLQBi1A): Fully autonomous

## Configuration

The configuration files are saved in the [config](config) folder. \
[spot_launch](launch/spot_launch.py)
contains lots of **if** statements, to deactivate parts of the program, for debugging.

## Contributing

If you would like to contribute, please feel free to open a pull request or reach out
to me via [e-mail](marco_alemao@hotmail.com). \
It will be nice to talk to other developers :smile:

## License

RustBuster is an open-source software. So just use it as you please :smile: