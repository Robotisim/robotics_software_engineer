### Simple plain Error Holding
```
                                       +------------------+
                                       |   GOAL POSITION  |
                                       | (goal_x, goal_y) |
                                       +------------------+
                                                |
                                                |
                                                v
                                  +-------------------------------+
                                  |   Calculate Distance to Goal  |
                                  |   and Angle to Goal Direction |
                                  +-------------------------------+
                                                |
                    +---------------------------+---------------------------+
                    |                                                       |
                    v                                                       v
          +------------------+                                    +-------------------+
          |   Proportional   |                                    |   Proportional    |
          |   Control for    |                                    |   Control for     |
          |   Angular Error  |                                    |   Distance Error  |
          |   (kp_angular)   |                                    |   (kp_linear)     |
          +------------------+                                    +-------------------+
                    |                                                       |
                    |                                                       |
                    v                                                       v
          +---------------------+                               +---------------------+
          |   Calculate Angular |                               |   Calculate Linear |
          |   Velocity based on |                               |   Velocity based on|
          |   error_angle and   |                               |   distance and     |
          |   kp_angular        |                               |   kp_linear        |
          +---------------------+                               +---------------------+
                    |                                                       |
                    +---------------------------+---------------------------+
                                                |
                                                v
                                  +-----------------------------+
                                  |   Apply Velocities to       |
                                  |   Robot (cmd_vel)           |
                                  +-----------------------------+

```

### Smooth Go to Goal

```
                                       +------------------+
                                       |   GOAL POSITION  |
                                       | (goal_x, goal_y) |
                                       +------------------+
                                                |
                                                |
                                                v
                                  +-------------------------------+
                                  |   Calculate Distance to Goal  |
                                  |   and Angle to Goal Direction |
                                  +-------------------------------+
                                                |
                                                |
        +-------------------------+             |             +--------------------------+
        |   Normalize Angular     |             |             |   Calculate Linear       |
        |   Error (error_angle)   |             |             |   Velocity Reduction     |
        +-------------------------+             |             |   Factor based on        |
                    |                           |             |   Angular Error          |
                    |                           |             +--------------------------+
                    |                           |                           |
                    v                           v                           v
          +------------------+        +---------------------+        +----------------+
          |   Proportional   |        |   Calculate Angular |        |   Calculate    |
          |   Control for    |        |   Velocity based on |        |   Linear       |
          |   Angular Error  |------->|   error_angle and   |------->|   Velocity     |
          |   (kp_angular)   |        |   kp_angular        |        |   based on     |
          +------------------+        +---------------------+        |   distance,    |
                                                                     |   kp_linear,   |
                                                                     |   and linear   |
                                                                     |   vel factor   |
                                                                     +----------------+

```