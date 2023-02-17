# RustBuster
<!---Automated Robotics Inspection of Offshore Platforms for Automated Maintenance-->
RustBuster is a project that uses the Spot robot from Boston Dynamics to inspect offshore oil platforms for rust and other forms of corrosion. The robot is programmed in Python and the Robot Operating System (ROS).
## Motivation

Offshore oil platforms are critical infrastructure that must be maintained to ensure their safety and longevity. Regular inspections are required to identify and address any signs of corrosion or other forms of degradation. However, these inspections can be dangerous and time-consuming for human inspectors, making it difficult to perform regular, thorough checks.

RustBuster was created to address this challenge by providing a safe, efficient, and automated solution for inspecting offshore oil platforms. The Spot robot is able to access difficult-to-reach areas and collect data on the condition of the platform, making it possible to perform regular and comprehensive inspections.
## Features

 * Spot robot equipped with cameras and sensors for data collection
 * Custom software developed in Python and ROS to control the robot and process data 
 * Automated inspection routines that allow the robot to survey offshore oil platforms for signs of corrosion and other forms of degradation

## Getting Started

These instructions will help you set up and use RustBuster for your own offshore oil platform inspections.
### Prerequisites

You will need the following software and hardware to use RustBuster:

 * Spot robot from Boston Dynamics
 * Computer with ROS2 and Python3 installed

### Installation
1. Clone the RustBuster and other required repositories to your computer:
   ```
   git clone https://github.com/Hakaino/RustBuster
   cd RustBuster
   git clone https://github.com/bdaiinstitute/spot_ros2.git
   ```
2. Install any necessary dependencies by running the following command:
   ````
   pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
   sudo apt install ros-$ROS_DISTRO-cartographer # used for 3D SLAM 
   rosdep install --from-paths src --ignore-src -r -y 
   ````
3. Build and source the project by running the following command:
    ```
    colcon build --symlink-install
    source install/setup.bash
    ```
### Usage
4. Connect the Spot robot to your computer and turn it on.
5. Launch the RustBuster program by running the following command:\
   ```
   ros2 launch rustbuster rustbuster_launch.xml
   ```
6. The robot will begin performing the inspection routine and collecting data. You can monitor the progress and view the data by using the ROS tools.

## Contributing

If you would like to contribute to the development of RustBuster, please feel free to open a pull request or reach out to us via email. We welcome all contributions, from fixing bugs to adding new features.
## License

RustBuster is open-source software, licensed under the MIT License.