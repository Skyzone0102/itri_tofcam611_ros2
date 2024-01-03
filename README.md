# ITRI TOFcam611 fall detection

This project uses the TOFcam611 to implement fall detection, which prevents the Autonomous Mobile Robot (AMR) from falling off the dock when operating nearby. This ROS2 node publishes a boolean message indicating whether a warning condition is met based on data from a TOFcam611 camera. If the message is true, it means there are two or more planes detected, and the AMR should stop completely to avoid a fall.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Parameters](#parameters)
- [Usage](#usage)
- [Contact](#contact)

## Prerequisites

Before you begin, ensure you have met the following requirements:

- You have installed the latest version of [ROS2](https://index.ros.org/doc/ros2/Installation/).
- You have a Linux machine. This project is not compatible with Windows or macOS.
- You have installed the necessary Python packages. This project uses `scikit-learn` and `numpy`.
- You have permission to access the serial port on your machine. If not, you can add your user to the `dialout` group using the command `sudo gpasswd --add ${USER} dialout` and then restart your machine.

## Installation

Follow these steps to install and run the project:

1. **Install ROS2**: Follow the instructions on the [official ROS2 website](https://index.ros.org/doc/ros2/Installation/) to install the latest version of ROS2.

2. **Clone the Repository**: Clone this repository into your ROS2 workspace's `src` directory.

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Skyzone0102/itri_tofcam611_ros2.git
   ```

3. **Install Python Dependencies**: Install the necessary Python packages using pip:

   ```bash
   pip install numpy scikit-learn
   ```

4. **Build the Package**: Navigate back to your ROS2 workspace directory and build the package:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select itri_tofcam611_ros2
   ```

5. **Source the Workspace**: Source your ROS2 workspace:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

6. **Run the Node**: Run the `TofCam611` node:

   ```bash
   ros2 run itri_tofcam611_ros2 read
   ```

## Parameters

This project uses the DBSCAN clustering algorithm, which has two key parameters:

- **EPSILON**: This is the maximum distance between two samples for them to be considered as in the same neighborhood. Adjusting this value changes how broadly the algorithm searches for nearby samples.

- **MIN_SAMPLE**: This is the number of samples in a neighborhood for a point to be considered as a core point. This includes the point itself. Adjusting this value changes how many samples are needed to form a dense region.

In addition to these, there is another parameter in `publisher.py`:

- **Plane Count Threshold (Line 54)**: The number 10 in this line is a threshold that decides how many labeled planes are considered to be one plane. Adjusting this value changes the sensitivity of the plane detection.

## Usage

This project is developed and tested on a Linux system using ROS2 Humble.

Follow these steps to run the program:

1. **Run the Node**: Open a terminal, navigate to your ROS2 workspace, and run the `TofCam611` node:

   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 run itri_tofcam611_ros2 read
   ```

2. **Listen to the Published Topic**: In a separate terminal, you can listen to the `/TofCam611_Warning` topic to see the boolean messages published by the `TofCam611` node:

   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic echo /TofCam611_Warning
   ```

Remember to replace `~/ros2_ws` with the path to your actual ROS2 workspace, and `humble` with your actual ROS2 distribution if it's not Humble.

## Contact

**GitHub Profile**: https://github.com/Skyzone0102

**Email**: michaelchou0102@gmail.com
