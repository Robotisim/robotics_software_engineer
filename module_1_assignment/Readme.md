# Module 1 Assignment: Introduction to OOP Concepts in C++ for Robotics

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

