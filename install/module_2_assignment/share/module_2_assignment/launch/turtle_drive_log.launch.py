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
    turtle_driver= Node(
        package ='module_2_assignment',
        executable = 'drive_turtle',
        name = 'turtle_driver',
        parameters = [{'cmd_vel_topic':'/turtle1/cmd_vel'}]
            
    )
    
    # # spawing the turtle 2nd.
    # spawn_turtle1 = ExecuteProcess(
    #     cmd = ['ros','service', 'call', '/spawn', '/turtlesim/srv/Spawn', "\"{x: 2.0, y: 1.0, theta: 1.75, name : 'turtle2'}\""],
    #     name = 'spawn_turtle1',
    #     shell = 'True'
    # )
    
    # turtle_driver2= Node(
    #     package ='drive_mobile_robot',
    #     executable = 'drive_turtle',
    #     name = 'turtle_driver2',
    #     parameters = [
    #         {'cmd_vel_topic':'/turtle2/cmd_vel'}
    #     ]
            
    # )
    return LaunchDescription([
        # we will call the launch name here
        # add the name of the node in order of their execution
        turtlesim # this will fire up the gui
        , 
        turtle_driver # this will run the turtle.
        ,
        # spawn_turtle1 # to spawn the turtle
    ])
