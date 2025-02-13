"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np

pose_x = 0
pose_y = 0
pose_theta = 0

# create the Robot instance.
robot = Supervisor()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1257 # ePuck wheel speed in m/s
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = 0
vR = 0

# Initialize gps and compass for odometry
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

def ramp_speed_func(curr_speed, target_speed, ramp_rate):
    return curr_speed + ramp_rate * (target_speed - curr_speed)

# use by doing ramp_speed(args)
ramp_speed = np.vectorize(ramp_speed_func)

# TODO: Find waypoints to navigate around the arena while avoiding obstacles
waypoints = [(-0.308, -0.247, -np.pi/2), (-0.308, -0.445, 0), (0.335, -0.415, np.pi/2), (0.325, -0.245,np.pi), (0.08, -0.08,np.pi), (0.045, -0.01, np.pi/2),
             (0.094, 0.056, 0), (0.325, 0.213, 0), (0.356, 0.262, np.pi/2), (0.321, 0.314, np.pi), (0.138, 0.42, 0), (-0.308, 0.427, 0), (-0.448, 0.3, 0), (-0.448, 0, 0)]
# Index indicating which waypoint the robot is reaching next
index = 0

# Get ping pong ball marker that marks the next waypoint the robot is reaching
marker = robot.getFromDef("marker").getField("translation")

def get_motor_speeds(dist_error,bearing_error,heading_error):
    theta_epsilon = 1e-2
    vR=0
    vL=0
    
    #if dist error is >0.1
        #if bearing error < theta_epsilon, drive straight
    if(dist_error>0.05):
        if(bearing_error>0 & bearing_error>theta_epsilon):
            vL = ramp_speed(vL, -MAX_SPEED / 4, 0.1)
            vR = ramp_speed(vR, MAX_SPEED / 4, 0.1)
        else:
            vL = ramp_speed(vL, MAX_SPEED / 4, 0.1)
            vR = ramp_speed(vR, -MAX_SPEED / 4, 0.1)
        #else, correct bearing error by turning
    #if dist error <=0.1, minimize heading error
    
    return vR, vL

controller_state = "turn_drive_turn_control"

while robot.step(SIM_TIMESTEP) != -1:
    # Set the position of the marker
    desired_x = waypoints[index][0]
    desired_y = waypoints[index][1]
    desired_theta = waypoints[index][2]
    marker.setSFVec3f([desired_x, desired_y, 0.01])

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # Read pose_x, pose_y, pose_theta from gps and compass
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]
    pose_theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])

    # TODO: controller

    # calculate distance error
    dist_error = np.sqrt((desired_x - pose_x) ** 2 + (desired_y - pose_y) ** 2)
    # calculate bearing error
    bearing_error = np.arctan2((desired_y - pose_y), (desired_x - pose_x)) - pose_theta
    while(bearing_error<-np.pi):
        bearing_error+=2*np.pi
    while(bearing_error>np.pi):
        bearing_error-=2*np.pi
    # calculate heading error
    heading_error = desired_theta - pose_theta

    print("Current error: [%5f, %5f, %5f]" % (dist_error, bearing_error, heading_error))

    if dist_error < 0.05:
        index += 1
        if index == len(waypoints):
            index=0

    # vR,vL = get_motor_speeds(dist_error,bearing_error,heading_error)

    if controller_state == "turn_drive_turn_control":
        if bearing_error > 0:
            vL = ramp_speed(vL, -MAX_SPEED / 4, 0.1)
            vR = ramp_speed(vR, MAX_SPEED / 4, 0.1)
        else:
            vL = ramp_speed(vL, MAX_SPEED / 4, 0.1)
            vR = ramp_speed(vR, -MAX_SPEED / 4, 0.1)

        if abs(bearing_error) < 0.05:
            if dist_error > 0.05:
                vL = ramp_speed(vL, MAX_SPEED, 0.1)
                vR = ramp_speed(vR, MAX_SPEED, 0.1)
            else:
                vL = ramp_speed(vL, 0, 0.1)
                vR = ramp_speed(vR, 0, 0.1)


    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)

    # turn_drive_turn_control state
