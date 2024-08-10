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
