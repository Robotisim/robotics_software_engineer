#include <iostream>
#include "motors.h"
void motors::moveForward(){
    std::cout<<"-> Driving Forward"<<std::endl;
}
void motors::moveReverse(){
    std::cout<<"->Driving Reverse"<<std::endl;
}
void motors::turnLeft() {
    std::cout << "->Turning left" << std::endl;
}

void motors::turnRight() {
    std::cout << "->Turning right" << std::endl;
}
