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
    # Calculate how far we are from the target as a percentage
    speed_diff = target_speed - curr_speed
    
    # Make ramp rate larger when closer to max/min speeds and smaller near zero
    # Using absolute values to handle both positive and negative speeds
    normalized_curr = abs(curr_speed) / MAX_SPEED
    dynamic_ramp = ramp_rate * (0.5 + normalized_curr)
    
    return curr_speed + dynamic_ramp * speed_diff

# use by doing ramp_speed(args)
ramp_speed = np.vectorize(ramp_speed_func)

# TODO: Find waypoints to navigate around the arena while avoiding obstacles
waypoints = [(-0.3, -0.25, -np.pi/2), (-0.308, -0.46, 0), (0.355, -0.415, np.pi/2), (0.325, -0.195,3*np.pi/4), 
             (0.065, -0.1,3*np.pi/4), (0, 0, np.pi/2), (0.09, 0.1, np.pi/4), (0.36, 0.218, np.pi/4), 
             (0.41, 0.28, np.pi/2), (0.33, 0.365, 3*np.pi/4), (0.1, 0.46, np.pi), (-0.348, 0.44, 5*np.pi/4), 
             (-0.47, 0.245, 3*np.pi/2), (-0.45, 0, 7*np.pi/4)]
# Index indicating which waypoint the robot is reaching next
index = 0

skip_waypoints = []

def get_next_index():
    """
    Get the next valid waypoint index while respecting skip_waypoints
    """
    global index, waypoints, skip_waypoints
    waypoints_length = len(waypoints)
    next_idx = index + 1
    if next_idx >= waypoints_length:
        next_idx = 0
    
    # Keep incrementing until we find a valid index
    while next_idx in skip_waypoints:
        next_idx += 1
        if next_idx >= waypoints_length:
            next_idx = 0
            
    return next_idx

# Get ping pong ball marker that marks the next waypoint the robot is reaching
marker = robot.getFromDef("marker").getField("translation")

def get_motor_speeds(dist_error,bearing_error,heading_error, vR, vL):
    global index
    theta_epsilon = 5e-2
    dist_epsilon = 1e-1
    ramp_delta = 0.1
    
    #if there is a significant distance error
    if(dist_error>dist_epsilon):
        #if there is a significant bearing error, turn to correct it
        if(np.abs(bearing_error)>theta_epsilon):
            if bearing_error > 0:
                vL = ramp_speed(vL, -MAX_SPEED / 4, ramp_delta)
                vR = ramp_speed(vR, MAX_SPEED / 4, ramp_delta)
            else:
                vL = ramp_speed(vL, MAX_SPEED / 4, ramp_delta)
                vR = ramp_speed(vR, -MAX_SPEED / 4, ramp_delta)
        #else, drive straight
        else:
            vL = ramp_speed(vL, MAX_SPEED, ramp_delta)
            vR = ramp_speed(vR, MAX_SPEED, ramp_delta)
    #if there is no dist and bearing error, minimize heading error
    elif(np.abs(heading_error)>theta_epsilon):
        if heading_error > 0:
            vL = ramp_speed(vL, -MAX_SPEED / 4, ramp_delta)
            vR = ramp_speed(vR, MAX_SPEED / 4, ramp_delta)
        else:
            vL = ramp_speed(vL, MAX_SPEED / 4, ramp_delta)
            vR = ramp_speed(vR, -MAX_SPEED / 4, ramp_delta)
    #all error is gone, continue to next checkpoint
    else:
        index += 1
        if index == len(waypoints):
            index=0
        #if index is in skip_waypoints, skip it
        while index in skip_waypoints:
            index += 1
            if index == len(waypoints):
                index=0
        print("waypoint #",index)
    #print("vr,vl: [%5f, %5f]"%(vR,vL))
    
    return vR, vL

controller_state = "turn_drive_turn_control"
controller_state = "test"
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
    
    # calculate heading error
    heading_error = desired_theta - pose_theta
        
    #alternate handling error: bearing to next waypoint
    desired_x = waypoints[get_next_index()][0]
    desired_y = waypoints[get_next_index()][1]      
    heading_error = np.arctan2((desired_y - pose_y), (desired_x - pose_x)) - pose_theta
    
    #fix theta out of bounds
    while(bearing_error<-np.pi):
        bearing_error+=2*np.pi
    while(bearing_error>np.pi):
        bearing_error-=2*np.pi
    while(heading_error<-np.pi):
        heading_error+=2*np.pi
    while(heading_error>np.pi):
        heading_error-=2*np.pi
    

    #print("Current error: [%5f, %5f, %5f]" % (dist_error, bearing_error, heading_error))


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
                
    elif controller_state == "test":
        vR,vL = get_motor_speeds(dist_error,bearing_error,heading_error, vR, vL)


    #print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)

    # turn_drive_turn_control state
