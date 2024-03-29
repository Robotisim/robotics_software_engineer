# These assignment-Tasks are not yet Finilized

### Assignments for Module #9 : Custom Interfaces
- Create all files in *module_9_assignment* package
### Assignment 1: Environmental Awareness and Adaptive Navigation

- **Custom Message Design**
    - Message Name: EnvironmentalData
    - Fields:
        - float64 temperature - Ambient temperature detected by a sensor.
        - float64 humidity - Ambient humidity level detected by a sensor.
        - bool is_obstacle_near - Indicates if an obstacle is detected nearby.
        - string navigation_status - Indicates the current navigation status (e.g., "navigating", "idle", "obstacle_detected").
        - float32 battery_level - Current battery level percentage of the robot.
Tasks
Custom Message Creation

Define the EnvironmentalData message in your ROS2 package, specifying the fields as described above.
Build the package to generate the necessary language-specific libraries for using the EnvironmentalData message in ROS2 nodes.
Sensor Data Aggregation Node

Implement a ROS2 node that simulates or reads actual sensor data (temperature, humidity, obstacle presence) and the robot's battery level.
Populate an EnvironmentalData message with these values and the current navigation status.
Publish the EnvironmentalData message at a regular interval.
Adaptive Navigation Node

Create another ROS2 node that subscribes to the EnvironmentalData messages.
Based on the received data, implement logic to adjust the TurtleBot3's navigation behavior. For example:
Slow down or alter the path if is_obstacle_near is true.
Return to a charging station if battery_level is below a threshold.
Adjust navigation strategy based on temperature and humidity levels, if those factors are relevant to your application scenario.