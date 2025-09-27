<h1 align="center">🗼 Perse Simulation (Webots) 🤖</h1>
<p align="center">Perse simulation environment</p>

## 📷 Screenshots

## 🔧 Install

This project is built to be used with the ROS 2 (humble) environment.

First, you must clone this repository inside a ros workspace, then install the dependencies using the following commands:

```bash
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

With all the dependencies installed, compile the workspace

```bash
colcon build --symlink-install
```

> If you do not have Webots already installed, in this step, you will be prompted to install


## 🎈 Intro

The environment is built upon Webots, an open source general purpose robotics simulator

In order to start the simulation, run the following command

```bash
ros2 launch sim_launch.py
```

> **Launch Arguments**
> - `odometry_variables`: Path to *odometry_variables.yaml* file, containing all target points


## 📁 Folder structure

- config/ - Configuration files
- protos/ - [Webots PROTO files](https://cyberbotics.com/doc/reference/proto) that describe all world elements.
- worlds/ - [Webots world files](https://cyberbotics.com/doc/reference/webots-world-files) for simulation
- meshes/ - Meshes used to display the robot
- resource/ - URDF file
- launch/ - ROS Launch files
- src/ - Webots controllers
