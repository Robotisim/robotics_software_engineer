
# Module # 5 : Control Systems and Dynamics ( V1.0 )
- Package Created : *control_systems*
    - PID , LQR  for go to goal
    - ros2_control : Different Controllers in multiple robot types
    - Fuzzy , Optimial , Non Linear Controls
- Questions
    - When to apply what?
---
## Lectures

- `a_module_intro`
- `b_applying_controllers_to_diff_drive`
    - Install :
        - sudo apt-get install ros-humble-gazebo-ros2-control
        - sudo apt-get install ros-humble-joint-trajectory-controller
        - sudo apt-get install ros-humble-velocity-controllers
        - sudo apt-get install ros-humble-position-controllers
    - Apply below to differential drive Robot
        - Poisition
        - Velocity Controller
    - Create launch file -> bring robot in rviz -> write  yaml file
    - Load sim -> controller manger -> bringup controller through cli
    - Send commands and discuss in rviz

    - *overlay* :
        - [screen] :
            - Explain how controllers work in ros2 -> low level + high level
            - Provides dependency package installations
            - End pay phir review overlay

- `c_linear_goal`
    - With and without Error Feedback
    - Then Move to the goal -> show oscillations
    - CD_linear goal  -> discuss in detail other comparison of KP value
    - *overlay* :
        - [screen] :
            -  How pid works
            - Control Terms Explanation ( set point , Oscillation , steady state error )
            - Show the graph
        - [ref] : Book Examples
        - [face] : Control System is fairly a big topic lets explore proportional controll in application for smooth robot behaviours
- `d_planer_gtg_seq_P`
    - Sequential_proportional
    - Problems
    - *overlay* :
        - [screen] :
            - what we will be doing here
            - Explain problems here and what can we do here

- `e_planer_gtg_factor_P`
    - Show linear vel reduction factor effect
    - Improving Code in 3 step overlays
    - *overlay* :
        - [screen] :
        - Face video record that in control system most people thik that If P is not workign simply apply ID and things will work but always you have to understand the problem in depth and then move look for a solution .
        - Represent Graphs of imporvments for 3 steps
            - basic sequential implementation
            - Independent proportional controllers
            - Most important -> angle normalization
            - Linear velocity clamping

- `f_imporve_line_following`
    - Get Line Following for mofule 4 and fix its code from bang bang controller to smoother output
    - Make it a faster Line Following
    - *overlay* :
        - [screen] : Explain the problem , how we will solve it


- `g_LQR_implementation`
    - explain jargons and relate to control
    - Hosla , it is hard and matrices -> but that is how actual algorithm work.
    - *overlay*
        - [screen] :
            - Explain intricacies of lqr , workflow diagrams
            - Refer some robotics book
        - [face] :
            - LQR is an important and why and why not PID ?
- `h_compare_all_algos_multi_tb3`
    - Spawn multiple tb3 in gazebo
    - paramterize goals and check smooth behaviours
    - *overlay* :
        - [face] :
            - Discussion on important control Jargons
        - [screen] : What we are going to do
- `i_robotic_arm_controllers`
    - Apply below to differential robotic arm
        - Effort controller
        - gripper_controllers/GripperCommandController
        - joint_trajectory_controller/JointTrajectoryController
    - *overlay* :
        - [screen] :
            - Explain how controllers work in ros2 -> low level + high level
            - Show rqt Graph

---
## Formating and linting
- clang-format -i  f_formate.cpp
- clang-tidy f_formate.cpp --fix
---