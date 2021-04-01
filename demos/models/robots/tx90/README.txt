
File tx90.urdf has been generated as follows:

  $rosrun xacro xacro tx90_macro.xacro > tx90.urdf

The original tx90_macro.xacro file and mesh files can be found in this package:
  https://github.com/ros-industrial/staubli_experimental.git

  - staubli_tx90_support/urdf/tx90_macro.xacro
  - staubli_tx90_support/meshes/tx90/visual
  - staubli_tx90_support/meshes/tx90/collision

The auxiliar files can be found in this package:
  https://github.com/janrosell/staubli.git

  - staubli_resources/urdf/common_materials.xacro
  - staubli_resources/urdf/common_colours.xacro

Changes done:
1) tx90_macro.xacro and common_materials.xacro have been edited to remove $package dependencies.
2) tx90_macro.xacro - removed ${prefix}
2) tx90_macro.xacro - added robot name
3) tx90_macro.xacro - removed links base, flange and tool0 and their fixed joints
