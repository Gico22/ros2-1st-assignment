# Assignment 1 – ROS2 Turtlesim Safety System

This project implements two ROS2 nodes that allow the user to control two turtles in `turtlesim` while ensuring collision and boundary safety.  
All safety behaviour is handled automatically through velocity reversal and pose monitoring.

---

## Nodes Overview

### 1. `ui` node  
A simple user-input interface that:
- Asks which turtle to move (`1` or `2`)
- Asks for linear and angular velocity
- Publishes the chosen velocity to `/turtleX/cmd_vel` for exactly 1 second
- Publishes the number of the moving turtle on the `moving_turtle` topic  
- Automatically stops the turtle after the 1-second motion

This node never checks safety conditions; it only sends the user’s command.

---

### 2. `distance` node  
A safety controller that continuously monitors:
- The distance between turtles (`≤ 0.5` is unsafe)
- Their position relative to the simulation boundaries (`< 1` or `> 10` is unsafe)
- The last commanded velocity of the moving turtle (to compute a reverse velocity)

Safety behavior:
- When a dangerous condition is detected  the flag `not_safe`is set as `True`
- While unsafe, the moving turtle receives `safety_twist` (reverse of the last user input)
- When the turtles return to a fully safe configuration (`distance > 1.2` AND both turtles inside `[1.2, 9.8]`), the moving turtle is stopped and `not_safe` becomes `False`

A single boolean flag controls the safety state, keeping the logic simple and efficient.

---

## Topics

### Published
- `/turtle1/cmd_vel`, `/turtle2/cmd_vel` — velocity commands  
- `moving_turtle` (`Int32`) — active turtle selected by UI  
- `distance` (`Float32`) — current distance between turtles  

### Subscribed
- `/turtle1/pose`, `/turtle2/pose`  
- `/turtle1/cmd_vel`, `/turtle2/cmd_vel` (used to compute reverse velocity)  
- `moving_turtle`  

---

## Running Instructions

From the workspace root:

```bash
cd ~/ros2_ws
colcon build
source install/local_setup.bash
```

Open four different terminals

1. Start turtlesim:
```bash
ros2 run turtlesim turtlesim_node
```

2. Start turtle_spawn:
```bash
ros2 run assignemnt1_rt turtle_spawn
```

3. Start safety node:
```bash
ros2 run assignemnt1_rt distance
```

4. Start UI node:
```bash
ros2 run ros2 run assignemnt1_rt ui
```

The turtle will move for 1 second, with automatic safety overrides.

---

## Expected Behavior
- If the turtles get too close, the moving turtle automatically receives a reverse velocity.
- If a turtle exits the allowed boundaries, it is pushed back inside using reverse velocity.
- Once the turtles are safely separated and inside the safe area, the node publishes a stop command and resets the not_safe flag.
- After safety resolution, the user regains full control of the turtles.

This implementation results in a responsive and robust safety layer on top of a simple UI-based control interface.
