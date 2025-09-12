# Kautham Models Structure

This repository organizes all resources required to define and create robots in [Kautham](https://github.com/iocroblab/kautham). The directory is structured to clearly separate **generic robot descriptions and macros** from **robot instances**, which can be either single robots or composite systems built from multiple descriptions.

This structure improves maintainability and reusability, making it easy to manage both individual robots and complex robot systems in Kautham.

## Directory Overview

- **robot_descriptions/**  
  This folder contains all the files needed to describe robots and their reusable macros (XACRO).
  - Use it to define the structure, configuration, and components of each robot, including:
    - **Macro files:** XACRO files for reusable and parameterized robot modules.
    - **Configuration files:** Kinematics, joint limits, physical, and visual parameters.
    - **Meshes:** 3D models for collision and visualization.
  > Use this folder to describe the *architecture and features* of each robot, and to provide modular building blocks for robot construction.

- **robot_instances/**  
  This folder is for creating specific robot instances for use in Kautham scenarios.
  - Each subfolder represents a concrete instance, which can be:
    - A direct instantiation of a robot defined in `robot_descriptions`.
    - A composite robot, created by combining several descriptions/macros (for example, a robot arm with a gripper).
  > Use this folder to define the *composition* of robots, including both single and composite systems.

## Usage Example

1. **Define a new robot or macro:**  
   Add its description files and macros to `robot_descriptions/`.

2. **Create a new instance:**  
   Add a folder to `robot_instances/` and create the necessary files for the new instance, referencing one or more descriptions/macros from `robot_descriptions/`.




