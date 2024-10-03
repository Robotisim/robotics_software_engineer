<<<<<<< HEAD
# Assignments for Module #1 : C++ from Robotics Prespective
- Create all files in *module_1_assignment* package
### Assignment 1: Simulating Sensors with Hardcoded Values
- **Objective**: Learn how to simulate sensor data using hardcoded values in C++.
- **Tasks**:
Create a C++ program that represents a robot equipped with temperature and distance sensors.
Use hardcoded values to simulate sensor readings (e.g., temperature fluctuations, distance to objects).
Print these values to the console with appropriate descriptions (e.g., "Temperature: 20°C", "Distance: 100cm").
- **Learning Outcome**: Apply C++ syntax to simulate basic robotics concepts.

### Solution :
    To make it work I followed following steps
    1. include the I/O stream
    2. using namespace std.
    3. create a function that take two argument (int *Templist, int size)
        - inside the function we use int *Templist
        - passing the argument as a pointer
    
    4. I ran a for loop iteration where it will reiterate the function till the i is equal to size.
    5. Inside the for loop I asked the function to publish the temperature for length of the array.
    6. Then I have another array. The will give the distance.
    7. In main function we instantiate the function pass actual array, giving it a acutal numerical distance.
# ************************************************************************
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
#### Solution for Assignment 2:
=======
# Module 1 Assignment: Introduction to OOP Concepts in C++ for Robotics
>>>>>>> temp-backup

## Objective

This assignment aims to introduce basic Object-Oriented Programming (OOP) concepts using C++ within a robotics context. You will develop a series of C++ programs that simulate robotic behavior, utilize sensor data, and demonstrate fundamental OOP principles.

## Tasks

### Task 1: Robot Class Implementation

- **Define a `Robot` class** with the following attributes:
  - `name`: The name of the robot.
  - `speed`: The speed of the robot.
  - **Physical Attributes:**
    - `weight`: The weight of the robot.
    - `size`: The size of the robot.
    - `number_of_sensors`: The number of sensors the robot has.

- **Methods for moving the robot:**
  - `moveForward()`: Simulate the robot moving forward.
  - `moveBackward()`: Simulate the robot moving backward.
  - `stop()`: Simulate stopping the robot.

- **Instantiate a `Robot` object** and simulate actions by invoking its methods.

- **Use namespaces** to define different robots. Ensure that each action is outputted to the console to demonstrate the robot's behavior.

### Task 2: Simulating Sensor Readings

- **Create a C++ program** that represents a robot equipped with temperature and distance sensors.

- **Simulate sensor readings** with hardcoded values:
  - Example for temperature: `Temperature: 20°C`
  - Example for distance: `Distance: 100cm`

- **Print these values** to the console with appropriate descriptions.

### Task 3: Sensor Library Design

- **Design a simple sensor library** that includes classes for different types of sensors:
  - `TemperatureSensor`: Class for handling temperature readings.
  - `DistanceSensor`: Class for handling distance measurements.

- **Use these classes in a main program** to simulate getting readings from sensors.

- **Create a single-class template** that can be used for multiple sensor types:
  - For `double` data
  - For `string` data
  - For `char` data

- **Ensure proper documentation** and use a `CMakeLists.txt` file for building the project.

## Learning Outcome

By completing this assignment, you will:
- Apply C++ syntax to simulate basic robotics concepts.
- Grasp fundamental OOP principles and understand their application in robotics software development.
----
## Submission Process

1. **Fork the Repository:**
   - Fork the `Robotics Software Engineer` repository, ensuring all branches are included in your fork.

2. **Clone Your Forked Repository:**
   ```bash
   git clone <your-forked-repo-url>
   cd robotics_software_engineer
   ```

3. **Create Files:**
   - Navigate to the `module_1_assignment` package.
   - Create files for each task as required.

4. **Document Your Work:**
   - Create a `README.md` file in the `module_1_assignment` package.
   - Provide details about the files you created.
   - Explain the commands required to run your code for each specific task.

5. **Submit Your Assignment:**
   - Push your changes to your forked repository.
   - Provide your repository link in the assignment submission text area.
   - **Note**: Ensure you press the "Start Assignment" button when you see the page (as it takes time to generate the pages).

6. **Wait for Review:**
   - Wait for the instructors to review your submission.

