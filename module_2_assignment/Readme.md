<<<<<<< HEAD
# How to solve the assignment
- Assignment ask to write a code that will make turtle in turtlesim move in circular motion or spiral motion. Also we need to be able to give user the authority to choose the whether the motion is circular or spiral. 

To achieve this pheonomenon we have to understand what sort of node we have to create. Since we will be commanding the motion and velocity, this implies we want to create a publisher node. Also we want to have condition if user choose between the circular and spiral motion for that we will use the if/else condition. And to make the user easier to choose we will pass the motion as a parameter. 

# How to create a code
- Step 1 : 
    * include all the neccesary libraries
    - include <chrono> : this is used for the time data type
    - include <funcitonal> : this is use
    - include <memory> : this is
    - include <string> we define the library for the use of string data type.
    
    Note : all the libraries inside the "<>" symbol are C++ default libraries
    where as the libraries ("") inside the double quotations are libraries of the ROS2. 
    The "rclcpp/rclcpp.hpp" help to establish the communication with the ROS2 workspace.
    The "std_msgs/msg/int16.hpp" and "geometry_msgs/msg/twist.hpp" are interface used to declare the publisher data type.

    The use of namespace std::chrono_literals allow us to write the time value without std::chrono literals in the code, which we  use while creating the "timer_" inside the code.

- Step 2: Now create the Class 
        while creating a class we always define the class
        here we use "class" at the beginning then name of the Class
        after that we ask the rclcpp for public level access to Node instances.

        Inside the class we make two section:
        * one accessible to public
      called "public"
        - here we declare the parameter
        - then we create a publisher and timer_

        * one not accesible to public
        called "private"
          - here we create timer_callback function that have the variable called the message that pass the data from publisher to the suscriber.
          - we define the data type using the similar wording to the data type we use to create the publisher.
           - the timer_callback function is passed in timer_ which is wall_timer that  publish the mesage, every 500 milliseconds. 

        - we also define the global variable as well as the publisher, walltimer we create with the name of the interfaces.

    - Step 3: Creating the main function
        - inside this section we inititate the rclcpp communication and make a sharedptr of our class. 
        - Also we ask the node to keep running until it is shutdown.
        - since we create a  int main we ask return the value 0, to end the session.

## Code to run 
- #### Launch file:=
    turtle_drive_log1.launch.py

- #### CPP file:=
    turtle_log_spiral.cpp
    
=======
# Module 2 Assignment: Developing Custom ROS 2 Nodes and Launch Files

## Objective

This assignment focuses on developing your ability to write custom ROS 2 nodes and utilize launch files for running multiple nodes simultaneously. You will create a custom ROS 2 node that controls the Turtlesim simulation and develop a launch file to run the simulation and node together.

## Tasks

### Task 1: Create a Custom ROS 2 Node

- **Develop a ROS 2 node** that makes the Turtlesim follow a unique pattern:
  - **Circle Movement:** The turtle should move in a circle with a radius that is provided as a user input.
  - **Logarithmic Spiral Movement:** The turtle should move in a logarithmic spiral pattern.

### Task 2: Develop a Launch File

- **Create a launch file** that starts the Turtlesim simulation and the custom ROS 2 node simultaneously.

- **Ensure proper documentation** of the node and launch file creation process, including the code and the results of executing the tasks.

### Task 3: Modify the Turtlesim Simulation Environment

- **Use existing Turtlesim services** such as `spawn` and `clear` to modify the simulation environment:
  - **Spawn 5 Turtlebots** with a single launch file, placing them diagonally from the top left to the bottom right.
  - **Drive the middle 3 turtles** back and forth continuously using ROS 2 services.

### Task 4: Modify Turtle Behavior with Parameters

- **Utilize ROS 2 parameters** to alter the behavior of the turtles:
  - **Change the speed** of the turtles dynamically during the simulation.

## Learning Outcome

By completing this assignment, you will:
- Understand how to develop and execute custom nodes in ROS 2.
- Learn the utility of launch files in managing the execution of multiple nodes in ROS 2.
- Learn how to interact with ROS 2 services to modify node behavior and simulation environments.
- Understand how to use ROS 2 parameters to control and alter the behavior of nodes in real-time.
---
## Submission Process

1. **Create Files:**
   - Navigate to the `module_2_assignment` package.
   - Create the required files for the custom ROS 2 node and launch file.

2. **Document Your Work:**
   - Create a `README.md` file in the `module_2_assignment` package.
   - Provide details about the files you created, including explanations of the code and the commands needed to run your custom node and launch file.

3. **Submit Your Assignment:**
   - Push your changes to your forked repository.
   - Provide your repository link in the assignment submission text area.
   - **Note**: Ensure you press the "Start Assignment" button when you see the page (as it takes time to generate the pages).

4. **Wait for Review:**
   - Wait for the instructors to review your submission.
>>>>>>> temp-backup
