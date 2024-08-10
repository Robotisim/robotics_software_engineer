
# Module # 3 : Transforms and URDF( V1.0 )
- Package Created : *custom_robots*
    - Transforms
    - URDF basic Shapes
        - Mobile Robot
        - Robotic Arm
    - URDF 3D Models
        - Explaining Physical Properties with Gazebo and effects

---
## Lectures

- `a_module_intro`
- `b_static_transforms`
    - Transforms relations explain
        - rotations
        - translations
    - Publish a dynamic transform
    - build and enviornment for robotic scene representation with transforms
- `c_urdf`
    - Create transforms using urdf file , makes life easier
- `d_robot_structure_tf2`
    - Robot Body design without Shapes
    - *overlay*
        - [screen] : Explain the connection between different joints
- `e_visual_body_shapes`
    - Add visual and collision tags to visualize robot
- `f_links_and_joints`
    - State publishing of static and dynamic things
    - *overlay* :
        - [screen] : Explain link and a joint -> then jsp and rsp
<!-- - `_robotic_arm_urdf`
    - Create URDF of robotic arm without Shapes
    - *overlay* :
        - [screen] : Lets take a look into Robotic Arm Example -->
- `g_gazebo_robot_drive`
    - Write Node to  drive Robot through Joint States
    - Add ros2_control Plugin
    - Add inertial parameters
    - Discuss parameters
    - drive robot
    - *overlay* :
        - [screen] : Gazebo is what 3D enviornmnt looks like
        - [face] :
            - Real robots have motor actuation here we have joint values on which we publish and robot joints move.

---
## Formating and linting
- clang-format -i  f_formate.cpp
- clang-tidy f_formate.cpp --fix
---