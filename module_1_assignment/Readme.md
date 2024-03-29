# Assignments for Module #1 : C++ from Robotics Prespective
- Create all files in *module_1_assignment* package
### Assignment 1: Simulating Sensors with Hardcoded Values
- **Objective**: Learn how to simulate sensor data using hardcoded values in C++.
- **Tasks**:
Create a C++ program that represents a robot equipped with temperature and distance sensors.
Use hardcoded values to simulate sensor readings (e.g., temperature fluctuations, distance to objects).
Print these values to the console with appropriate descriptions (e.g., "Temperature: 20°C", "Distance: 100cm").
- **Learning Outcome**: Apply C++ syntax to simulate basic robotics concepts.
### Assignment 2: Introduction to Object-Oriented Programming (OOP)
- **Objective**: Introduce basic OOP concepts using C++ within a robotics context.
- **Tasks**:
Define a Robot class with attributes
    - name
    - speed
    - Physical ( weight , size , number of sensors )
Methods for moving
    - moveForward
    - moveBackward
    - stopping.
Instantiate a Robot object and simulate actions by invoking its methods.
Utilize namespaces for defining different robots.
Output each action to the console to show the robot's behavior.
- **Learning Outcome**: Grasp OOP principles and their application in robotics software development.

### Assignment 3: Creating Custom Libraries for Robotics Components
- **Objective**: Learn how to create and use custom C++ libraries for reusable robotics components.
- **Tasks**:
Design a simple sensor library that includes classes for different types of sensors
    - TemperatureSensor
    - DistanceSensor
Use these classes in a main program to simulate getting readings from sensors.
Ensure proper documentation and use CMakeLists for building the project.

- Create Single Class Template to be utilized for multiple Sensors of different types
    - Double data
    - String Data
    - Character data
- **Learning Outcome**: Understand how to organize code into reusable libraries and compile them using CMake.