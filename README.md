# Waypoint Navigation with Obstacle Avoidance

This project implements intelligent waypoint navigation using the ePuck robot platform in the Webots simulator, featuring two different navigation controllers, position tracking options, and dynamic error correction.

## Project Structure
```
Lab 3/
├── README.md
├── worlds/                # Webots world files
├── controllers/           
│   └── lab3_soln/         # Robot controller implementation
└── ...                    # misc files
```

## Navigation Controllers

The controller implements two distinct navigation approaches that can be selected at runtime:

### 1. Turn-Drive-Turn Controller (`turn_drive_turn_control`)
- Sequential approach: first turn to face target, drive straight, then turn to desired heading
- Prioritizes bearing correction before applying forward movement
- Good for precise navigation with distinct movement phases
- Simple control logic with clear state transitions
- Uses `get_motor_speeds_part2()` function

### 2. Proportional Controller (`proportional_controller`)
- Simultaneous control of all error components
- Uses dynamic proportions to balance distance, bearing, and heading errors
- Applies proportional weighting based on error magnitudes
- More fluid, natural movement with continuous adjustments
- Uses `get_motor_speeds_part3()` function with sophisticated proportional calculations

## Positioning Systems

The robot can track its position using either:

1. **External Sensors (GPS/Compass)**
   - Higher accuracy positioning using simulator's ground truth
   - Less computational complexity
   - Not affected by accumulating errors

2. **Internal Odometry**
   - Uses wheel encoder information to estimate position
   - Implements midpoint integration for pose updates
   - More realistic simulation of real-world robots
   - Calculated via `update_odometry()` function

## Key Features

- **Dynamic Waypoint Management**: Predefined waypoints with ability to skip specific points
- **Error Handling**: Calculates and corrects three types of errors:
  - Distance error: Linear distance to target point
  - Bearing error: Angular difference to face target point
  - Heading error: Angular difference to desired orientation
- **Speed Ramping**: Implements smooth velocity transitions to prevent jerky movements
- **Proportional Control**: Dynamically adjusts control inputs based on error magnitudes
- **Visual Debugging**: Uses a marker to visualize the current target waypoint

## Error Correction Approach

The system calculates three key errors at each timestep:
- **Distance Error**: Euclidean distance to the next waypoint
- **Bearing Error**: Angular difference between robot's heading and direction to waypoint
- **Heading Error**: Difference between robot's current orientation and target orientation

The controller continuously adjusts wheel velocities based on these errors using proportional control with dynamically weighted priorities.

## Usage

1. Open the Webots world file
2. Select controller mode by changing the `controller_state` variable:
   ```python
   controller_state = "turn_drive_turn_control"  # Sequential movement
   # OR
   controller_state = "proportional_controller"  # Simultaneous control
   ```
3. Modify waypoints list to create new navigation paths
4. Add indexes to `skip_waypoints` list to bypass specific waypoints
5. Run the simulation and observe robot's navigation behavior
