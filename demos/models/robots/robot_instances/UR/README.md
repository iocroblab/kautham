# Usage of Universal Robots Description Files in Kautham

This document explains how to use the adapted Universal Robots description files in Kautham, focusing on the requirements for macro usage and project setup.


## Adaptations for use in Kautham

To enable integration and usage of these files in Kautham (outside of a ROS environment), some modifications have been made regarding path handling and file inclusion, especially to make mesh and configuration file paths relative instead of relying on the ROS package system (`find package`).


### Arguments for Configuration Paths

Arguments have been defined for configuration and robot description paths, allowing greater flexibility:

```
<xacro:arg name="robot_description_path" default="../../robot_descriptions/UR"/>
<xacro:include filename="$(arg robot_description_path)/ur_macro.xacro"/>
```

Various parameter files are also passed as arguments using relative paths:

```
<xacro:arg name="joint_limit_params" default="$(arg robot_description_path)/config/$(arg ur_type)/joint_limits.yaml"/>
<xacro:arg name="kinematics_params" default="$(arg robot_description_path)/config/$(arg ur_type)/default_kinematics.yaml"/>
<xacro:arg name="physical_params" default="$(arg robot_description_path)/config/$(arg ur_type)/physical_parameters.yaml"/>
<xacro:arg name="visual_params" default="$(arg robot_description_path)/config/$(arg ur_type)/visual_parameters.yaml"/>
```


## Important: Set `force_abs_paths` to `true`

**You must use**
```
<xacro:arg name="force_abs_paths" default="true" />
```
**and not set it to `false`.**  
This ensures that all mesh and configuration paths are resolved correctly using the relative paths required by Kautham. Setting `force_abs_paths` to `false` will result in unresolved resource paths outside of ROS.

## Usage

Make sure the folder `robot_descriptions/UR` contains a copy of the official Universal Robots ROS2 Description files with the indicated adaptations.

## Supported Models

Possible values for `ur_type` include:
- `ur3`, `ur3e`, `ur5`, `ur5e`, `ur7e`, `ur10`, `ur10e`, `ur12e`, `ur16e`, `ur15`, `ur20`, `ur30`


## Reference

- [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

**Note:** This README describes the changes made to adapt the official files for flexible and portable usage within Kautham while maintaining compatibility with their original structure.


## See Also

For details about the macro implementation and the modifications made to the original Universal Robots files, refer to [Modifications to the Universal Robots Description Macro for Kautham](./../../robot_descriptions/UR/README.md).