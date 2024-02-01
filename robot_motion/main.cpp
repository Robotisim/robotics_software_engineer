
#include <iostream>
#include <string>
#include "motors.h"
#include "camera.h"
#include "lidar.h"
#include <memory>

int main(){

    motors motor_obj;
    camera camera_obj;
    // lidar lidar_obj;
    std::shared_ptr<lidar> lidar_obj_ptr = std::make_shared<lidar>();


    std::string command;

    while(true){
        std::cout <<"Enter your driving Command "<<std::endl;
        std::cin>>command;


    if (command == "forward"){
        motor_obj.moveForward();
    }else if (command == "left"){
        motor_obj.turnLeft();
    }else if (command == "right"){
        motor_obj.turnRight();
    }else if (command == "reverse"){
        motor_obj.moveReverse();
    }else if (command == "camera"){
        camera_obj.sensor_output();
    }else if (command == "lidar"){
        lidar_obj_ptr->sensor_output();
    }
    else {
        std::cout<< " Invalid command " << std::endl;
    }
    }

    return 0;
}