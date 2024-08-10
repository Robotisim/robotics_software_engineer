# Module 3 Assignment: URDF and Robot Creation in ROS 2

## Objective

This assignment focuses on understanding and applying the concepts of URDF (Unified Robot Description Format) to create custom robots in ROS 2. You will design and build a robotic arm with multiple degrees of freedom (DOF), integrate it with a mobile platform, and create an Ackerman drive system.

## Tasks

### Task 1: Create a Custom Transform Tree

- **Design a robotic arm with 3 DOF** using URDF:
  - **Define the transform tree** for the robotic arm without including any visualization tags. Focus solely on creating the correct transforms for the arm's joints.
  - **Do not include any visual elements** at this stage—only the transforms should be defined.

### Task 2: Add Joints and Visual Elements

- **Enhance the robotic arm** you created earlier by adding joints:
  - **Finger Joints:** Use prismatic joint types for the fingers.
  - **Base Joint:** The base joint should be of the continuous type.
  - **All Other Joints:** Set these as revolute joints.

- **Add visualization tags** to your robot's URDF to create the body, primarily using cylinder shapes for simplicity.

### Task 3: Build a Mobile Manipulator

- **Integrate the robotic arm** with a mobile robot platform:
  - **Place the robotic arm** on top of a differential drive robot.
  - **Connect the arm** using the `base_link` of the differential drive robot.

- **Create an Ackerman Drive System:**
  - **Design a car-like robot structure** that represents the front axle rotations for turning, simulating an Ackerman steering mechanism.

## Learning Outcome

By completing this assignment, you will:
- Learn to create custom robots for simulations using URDF.
- Understand how to define and manipulate joints and transforms in URDF.
- Gain experience in building and simulating mobile manipulators and drive systems in ROS 2.
----
## Submission Process

1. **Create Files:**
   - Navigate to the `module_3_assignment` package.
   - Create the required URDF files to implement the tasks, including the robotic arm and mobile manipulator designs.

2. **Document Your Work:**
   - Create a `README.md` file in the `module_3_assignment` package.
   - Provide details about the files you created, including explanations of the URDF structure and the commands needed to visualize your robots in ROS 2.

3. **Submit Your Assignment:**
   - Push your changes to your forked repository.
   - Provide your repository link in the assignment submission text area.
   - **Note**: Ensure you press the "Start Assignment" button when you see the page (as it takes time to generate the pages).

4. **Wait for Review:**
   - Wait for the instructors to review your submission.
