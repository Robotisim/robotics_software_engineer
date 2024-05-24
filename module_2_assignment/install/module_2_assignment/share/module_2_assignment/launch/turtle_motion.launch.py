from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    #define launch here

    # 1st we will call gui
    turtlesim = Node(
        package ='turtlesim',
        executable = 'turtlesim_node',
        name = 'turtlesim'
    )

    # 2nd defining the driver
    turtle_driver1= Node(
        package ='module_2_assignment',
        executable = 'turtle_motion',
        name = 'turtle_driver',
        parameters = [{'motion_type':'circle'}]
            
    )
    
    return LaunchDescription([
        # we will call the launch name here
        # add the name of the node in order of their execution
        turtlesim # this will fire up the gui
        , 
        turtle_driver1 # this will run the turtle.

    ])
