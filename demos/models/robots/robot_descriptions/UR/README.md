# Modifications to use the Universal Robots Description Files in Kautham

This document details the changes made to the official Universal Robots description files from the [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) repository to adapt them for use in Kautham.  
It focuses on modifications to ensure compatibility outside of a ROS environment.

## Conditional Creation of the `base_joint`

To allow the robot description to be used both as a standalone model (with `base_link` as the root) and as a subcomponent attached to an external link, the creation of the `base_joint` has been made conditional.  
This is achieved by wrapping the joint definition in a conditional block that checks if the `parent` parameter is set:

```
<xacro:if value="${parent != ''}">
    <joint name="${tf_prefix}base_joint" type="fixed">
        <xacro:insert_block name="origin" />
        <parent link="${parent}" />
        <child link="${tf_prefix}base_link" />
    </joint>
</xacro:if>
```

- **If `parent` is specified**, the joint is created, fixing the robot to the given parent link.
- **If `parent` is not specified or is empty**, the joint is omitted, and `base_link` becomes the root of the kinematic tree.

This modification increases the flexibility of the robot description.

## Macro Adaptation for Relative Paths

The original macro for obtaining mesh paths relied on ROS package resolution. To ensure compatibility outside of ROS, the macro has been modified to support relative paths when `force_abs_paths` is set to `true`:

```
<xacro:macro name="get_mesh_path" params="name:=^ type:=^" >
    <xacro:get_visual_params />
    <xacro:if value="${force_abs_paths}">
        <xacro:property name="mesh" value="./../../robot_descriptions/UR/${visual_params['mesh']['path']}" scope="parent"/>
    </xacro:if>
    <xacro:unless value="${force_abs_paths}">
        <xacro:property name="mesh" value="package://${visual_params['mesh']['package']}/${visual_params['mesh']['path']}" scope="parent"/>
    </xacro:unless>
</xacro:macro>
```

- **When `force_abs_paths` is `true`**, the macro uses a relative path to locate mesh files within the project structure.
- **When `force_abs_paths` is `false`**, the macro uses the ROS package URI.

## Relative Inclusion of XACRO Files

To further support portability, XACRO file inclusions have been changed to use relative paths:

```
<xacro:include filename="./ur_common.xacro" />
```

This ensures all dependencies are resolved correctly, regardless of the working directory.

**These modifications ensure that all resources can be found without relying on ROS package resolution.**


## Reference

- [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

**Note:** This README describes only the changes made to the macro and file inclusions to enable flexible and portable usage within Kautham, while maintaining compatibility with the original structure of the official files.


## See Also

For usage instructions and integration details in Kautham, refer to [Usage of Universal Robots Description Files in Kautham](./../../robot_instances/UR/README.md).