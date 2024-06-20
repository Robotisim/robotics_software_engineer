# Assignments for Module #3 : Robot structure with URDF
- Create all files in *module_3_assignment* package



### Assignment 1: Robotic Arm Creation using TF
- **Tasks**:
Create a custom transform tree for a
    - Robotics arm of 3 DOF without body ( only transforms)
    - visualize it in rviz + utilize joint state publisher GUI to see transforms
    - You should not have any visualize tag filled yet.

Solution:

code to run : scara_robot.urdf
launch file : scara_robot.launch.py

syntax
$ ros2 run package_name scara_robot.launch.py

Idea behind the building the code

create a diff_drive
then use that as base to build the scara robot.

also same code is used for assignment 2 so we will not use robot model for this case. only TF frame.

### Assignment 2: Joints Understanding
- **Tasks**:
Add joints to the same Robotic Arm that you created earlier
    - Finger joints with prismatic joint type
    - Have base joint as continous
    - All other joints should be Revolute
    - Add visualize tag to your robot urdf and create body mostly using cylinders

Solution 
code to run : scara_robot.urdf
launch file : scara_robot.launch.py

syntax
$ ros2 run package_name scara_robot.launch.py

Idea behind the building the code

create a diff_drive
then use that as base to build the scara robot.

for assignment 2 , we will use robot model for this case. not only TF frame.
### Assignment 3: Building Mobile Manupilator
- **Tasks**:
Put your robotic arm on top of different drive robot
    - Connect using base_link of diff bot.

Solution

code to run : ackerman_drive.urdf
launch file : ackerman_robot.launch.py

same like differential driver we will be using two contionus wheel at the front with the parent link with steering which will steer the wheel.


