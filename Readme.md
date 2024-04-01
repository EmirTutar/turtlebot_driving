# AMR Final Projekt

This repository contains the source code for a ROS project that enables a mobile robot to autonomously navigate to predefined goals within an environment. It utilizes the ROS `move_base` package for path planning and navigation, integrating features like clearing the costmap, handling unreachable goals, and considering "Helper Goals" for complex navigation tasks. For detailed information and further insights into the software architecture and design, visit our [Wiki](https://fbe-gitlab.hs-weingarten.de/stud-amr/2023-ws-amr/jw-213685_tier4/-/wikis/home).

## Getting Started

To get this project up and running, ensure that ROS (Robot Operating System) and required dependencies, such as the `move_base` package, are installed. Copy this repository into the `src` folder of your Catkin workspace and compile the workspace with `catkin_make`. Make sure your robot or simulated environment is properly set up and the relevant ROS topics and services are available.

### Installation

- ROS Noetic (other versions may need testing)
- A working `move_base` setup
- Access to a mobile robot or a simulated environment

## Problems and Solutions
    
- **Unreachable Goals:**
If the robot has difficulties reaching certain goals, it might be due to the costmap. Try clearing the costmap by calling the appropriate service.

- **Narrow Passages and Walls:**
Should narrow passages or walls lead to "false" obstacles, adjusting the configuration of your sensors and the parameters of the `move_base` package can help, especially the costmap configurations. Lowering the inflation rate in the costmap parameters can allow the robot to navigate closer to actual obstacles without marking them as impassable. However, be cautious as setting the inflation rate too low may increase the risk of collisions. Careful calibration and extensive testing are advised to find the optimal setting for your specific robot platform and environment.


## Contact Us

| Name                  | Email                                      |
| ------                | ------                                     |
| Emircan Tutar         | emircan.tutar@hs-weingarten.de             |
| Jonathan Wekesser     | jonathan.wekesser@hs-weingarten.de         |
