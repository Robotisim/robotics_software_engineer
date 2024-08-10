# Module 7 Assignment: ROS 2 Mapping with SLAM

## Objective

This assignment focuses on the practical application of SLAM (Simultaneous Localization and Mapping) in ROS 2. You will create a 2D LIDAR-based map, explore the necessary inputs and outputs for 2D and 3D mapping tools, and explain the mapping algorithms in your own words.

## Tasks

### Task 1: Create a 2D LIDAR-Based Map

- **Objective:** Using the knowledge gained from the lectures, manually create a maze in Gazebo and generate a 2D map using a LIDAR sensor and the SLAM toolbox.

  - **Subtasks:**
    1. **Set Up the Maze Environment:**
       - Manually create a maze in Gazebo using the available tools.
       - Ensure that the maze is complex enough to demonstrate the capabilities of the SLAM algorithm.

    2. **Perform 2D Mapping:**
       - Use the SLAM toolbox to generate a 2D map of the maze using TurtleBot3 equipped with a LIDAR sensor.
       - Save the generated map and visualize it in RViz.

    3. **Document the Process:**
       - Provide a step-by-step explanation of how you set up the maze, configured the SLAM toolbox, and generated the map.

### Task 2: Understand Inputs and Outputs for 2D and 3D Mapping

- **Objective:** Explore and document the necessary inputs, outputs, and frames required for both 2D and 3D mapping using the SLAM toolbox and the RTAB-Map package.

  - **Subtasks:**
    1. **2D Mapping with SLAM Toolbox:**
       - Identify and document the required inputs (e.g., LIDAR data, odometry) and outputs (e.g., map data, tf frames).
       - Explain the role of each input and output in the mapping process.

    2. **3D Mapping with RTAB-Map:**
       - Identify and document the required inputs (e.g., RGB-D camera data, odometry) and outputs (e.g., point clouds, 3D map data, tf frames).
       - Explain how the inputs are processed and how the outputs are generated.

    3. **Compare 2D and 3D Mapping:**
       - Provide a comparison between 2D and 3D mapping in terms of complexity, accuracy, and the type of environments each is best suited for.

### Task 3: Explain the Mapping Algorithm (Gmapping)

- **Objective:** Explain in simple terms how the Gmapping algorithm works for creating maps.

  - **Subtasks:**
    1. **Simplified Explanation:**
       - Write a brief explanation of the Gmapping algorithm, focusing on the key concepts such as particle filters, map updating, and handling sensor noise.

    2. **Relate to Practical Application:**
       - Relate your explanation to the practical steps you took in Task 1 to create the 2D LIDAR-based map.
       - Highlight how Gmapping contributes to building an accurate and reliable map.
---
## Submission Process

1. **Create Files:**
   - Navigate to the `module_7_assignment` package.
   - Create the required files for the maze setup, SLAM configuration, and documentation.

2. **Document Your Work:**
   - Create a `README.md` file in the `module_7_assignment` package.
   - Provide details about the files you created, including explanations of the setup process, inputs/outputs for SLAM, and the Gmapping algorithm.

3. **Submit Your Assignment:**
   - Push your changes to your forked repository.
   - Provide your repository link in the assignment submission text area.
   - **Note**: Ensure you press the "Start Assignment" button when you see the page (as it takes time to generate the pages).

4. **Wait for Review:**
   - Wait for the instructors to review your submission.

## Learning Outcome

By completing this assignment, you will:
- Gain hands-on experience with creating maps using 2D LIDAR and 3D RGB-D sensors.
- Understand the necessary inputs and outputs for successful mapping in both 2D and 3D environments.
- Develop the ability to explain mapping algorithms like Gmapping in simple terms and relate them to practical applications.
