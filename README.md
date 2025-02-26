# Waypoint Navigation with Obstacle Avoidance

This project implements waypoint navigation using the ePuck robot platform in the Webots simulator. It features proportional control, speed ramping, odometry tracking, and dynamic waypoint skipping.

## Project Structure
```
Lab 3/
├── README.md
├── worlds/                # Webots world files
├── controllers/           
│   └── lab3_soln/         # Robot controller implementation
└── ...                    # misc files
```

## Components

### Controller
- **Waypoint Navigation**: The robot navigates through predefined waypoints using either GPS or built-in odometry
- **Proportional Control**: Adjusts motor speeds based on distance, bearing, and heading errors
- **Speed Ramping**: Smoothly transitions motor speeds to avoid abrupt changes
- **Odometry Tracking**: Uses midpoint integration to update the robot's position and orientation
- **Dynamic Waypoint Skipping**: Skips waypoints dynamically if they are marked to be skipped
- **Error Handling**: Corrects distance, bearing, and heading errors to ensure accurate navigation
- **Console Output**: Prints navigation information, including current errors and waypoint indices

## Usage
1. Open the Webots world file
2. The controller will automatically:
   - Navigate through waypoints using either GPS or odometry
   - Adjust motor speeds using proportional control and speed ramping
   - Skip waypoints dynamically if needed
   - Print navigation information in the console
