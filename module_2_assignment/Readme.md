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
    