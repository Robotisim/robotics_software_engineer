# Module 5 Assignment: Enhancing Robot Control for Smooth Motion

## Objective

This assignment is focused on enhancing the control of robots to achieve smoother motion and improved efficiency. You will work on improving a camera-based line-following system, introducing a PI controller, and optimizing the TurtleBot3's path to a goal with minimal energy consumption. Additionally, you will add a position controller to a robotic arm URDF model.

## Tasks

### Task 1: Improve Camera-Based Line Following

- **Enhance the line-following algorithm** from the previous module:
  - Improve the **speed and smoothness** of the line-following behavior by introducing a PI (Proportional-Integral) controller.
  - **Document the improvement** in your robot’s behavior, including the parameters used and their effects.

### Task 2: Optimize Goal Selection and Path Planning

- **Select a goal** for the TurtleBot3 based on the criterion of minimal energy consumption:
  - **Energy consumption** should be evaluated based on the distance to the goal.
  - Select the shortest path to the goal and plan the TurtleBot3's movement accordingly.

- **Develop a ROS 2 node** that plans and executes the TurtleBot3’s movement to the selected goal with minimal energy consumption.

### Task 3: Add a Position Controller to Robotic Arm URDF

- **Extend the URDF model** of the robotic arm created in the previous module:
  - Add a **position controller** to manage the arm's movements accurately.
  - Ensure the URDF file is correctly configured to simulate the position control behavior in a ROS 2 environment.
Based on your description, here's a proposed task for LQR with a focus on visualizing points in RViz and analyzing the effects of different Q and R values on the robot's behavior during multi-goal following:

### Task 4: Implement and Visualize LQR for Multi-Goal Following

- **Objective:** Enhance the multi-goal following behavior of TurtleBot3 using LQR and analyze the impact of different Q and R matrix values on the robot's path and performance.

  - **Subtasks:**

    a. **Visualize Goals and Path in RViz:**
       - Implement visualizations in RViz to display the positions of the goals and the robot’s path.
       - Add markers for each goal and a trail of points to represent the robot's odometry during the path following.

    b. **Experiment with Different Q and R Values:**
       - Test the LQR controller with three different sets of Q and R matrices.
       - Record and compare how these values affect the robot's behavior, particularly in terms of stability, speed, and smoothness when transitioning between goals.

    c. **Document and Analyze the Results:**
       - Provide a detailed report on the performance of the LQR controller with different Q and R settings.
       - Include visualizations from RViz to illustrate the robot’s path and goal positions for each scenario.
       - Discuss which set of Q and R values provided the best balance between responsiveness and smoothness in the robot’s motion.

---
### Submission Process

1. **Create Files:**
   - Navigate to the `module_5_assignment` package.
   - Create the required files for the improved line-following, goal selection, and position control tasks.

2. **Document Your Work:**
   - Create a `README.md` file in the `module_5_assignment` package.
   - Provide details about the files you created, including explanations of the code and the commands needed to run your simulations and controllers.

3. **Submit Your Assignment:**
   - Push your changes to your forked repository.
   - Provide your repository link in the assignment submission text area.
   - **Note**: Ensure you press the "Start Assignment" button when you see the page (as it takes time to generate the pages).

4. **Wait for Review:**
   - Wait for the instructors to review your submission.

## Learning Outcome

By completing this assignment, you will:
- Learn to control robots to create smoother and more efficient motion.
- Enhance your understanding of using controllers and path planning algorithms to optimize robot behavior.

