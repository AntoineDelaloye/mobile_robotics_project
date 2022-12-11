# Import tdmclient Notebook environment:
import tdmclient.notebook
import numpy as np
import PathPlanning
from ekf import ExtendedKalmanFilter
from PathPlanning import Global_navigation
import math

# Constants 
obstThrL = 500      # low obstacle threshold to switch state 1->0
obstThrH = 900      # high obstacle threshold to switch state 0->1
obstSpeedGain = [0.06, 0.04, 0, -0.04, -0.06]  # gain for motor speed necessary to avoid obstacle

# Variable initialization 
turn    = -1           # Choice of turning when an obstacle is detected: 0 for counter-clockwise and 1 for clockwise 
spLeft  = 50         
spRight = 50
current_target_idx = 0     # Index stored of the nex target 
current_plan = 0           # 0=A* algorithm, 1=obstacle avoidance

def obstacleAvoidance(position_x, position_y, prox_value, path):
    """
    This function is used to store the position when a local obstacle is detected.
    :param position_x: X coordinate of the current position 
    :param position_y: Y coordinate of the current position
    :param prox_value: proximity sensors value
    :param path: matrix containing all the coordinates calculated from A star algorthm 
    """
    
    # store the position from path before a local obstacle is detected
    saved_pos = checkPlan(position_x, position_y, prox_value)
    saved_pos_idx = 0

    # store the index of the kalman_position from path before a local obstacle is detected
    saved_pos_idx = 0
    for i in range(len(path)):
        if (path[i] == saved_pos).all():
            saved_pos_idx = i        
    return saved_pos_idx

def checkPlan(position_x, position_y, prox_value):
    """
    Check if the plan of the Thymio has changed, which indicate whether to follow the path planning 
    from the A* algorithm or to avoid a local obstacle 
    :param position_x: X coordinate of the current position 
    :param position_y: Y coordinate of the current position
    :param prox_value: proximity sensors value
    :return saved_pos: position of the Thymio before finding an obstacle 
    """
    global current_plan, obstThrH, obstThrL

    if current_plan == 0: 
        # switch from goal tracking to obst avoidance if obstacle detected
        if (max(prox_value[0:5]) > obstThrH):
            current_plan = 1
            saved_pos = [position_y, position_x]                        
            return saved_pos
    elif current_plan == 1:
        # switch from obst avoidance to goal tracking if obstacle got unseen 
        if max(prox_value[1:6]) < obstThrL:
            current_plan = 0
            saved_pos = 0
            return saved_pos
        
def motionControl(kalman_position, path, square_size, prox_value):
    """
    Check if the plan of the Thymio has changed, which indicate whether to follow the path planning 
    from the A* algorithm or to avoid a local obstacle 
    :param kalman_position: a vector containing coordinates x and y and angle of the Thymio inside the map
    :param path: contains all the coordinates of the optimal path from the start position to the goal 
    :param square_size: size of one square from the grid map
    :param prox_value: proximity sensors value
    :return spRight: speed of the right motor 
    :return spLeft: speed of the left motor
    """
    global obstSpeedGain, current_plan, current_target_idx, spLeft, spRight, turn

    thymio_speed = 100    
    range_angle = 10     # acceptable error angle in degrees for deciding if the direction of the Thymio is good 
    turn_factor = 3      # experimental gain for turning

    position_x, position_y, position_angle = kalman_position
    position_angle = position_angle*180/np.pi    # conversion of the angle from radians to degrees
    
    # set the right speed of the motors depending on the situation
    if current_plan == 0:
        saved_pos_idx = obstacleAvoidance(position_x, position_y, prox_value, path)
        if current_plan:
            return(0,0)
        
        if current_target_idx >= path.shape[0]:
            current_target_idx = path.shape[0]-1
            return (0,0)
        
        # stop when the goal is reached
        if (np.abs([position_y, position_x]-path[current_target_idx])<= np.array([1,1])).all():
            current_target_idx += 1
            if current_target_idx >= path.shape[0]:
                current_target_idx = path.shape[0]-1
                return (0,0)
            
        # calculate the target angle in degrees using Thymio position and next target position.
        targetAngle = math.atan2((path[current_target_idx][0] - position_y),(path[current_target_idx][1] - position_x))*180/np.pi %(360)
        # difference of the angle between Thymio orientation and taget angle
        diff = position_angle - targetAngle     
        
        # maintain the difference in a range [-180,180] 
        if (diff>180) : diff = diff-360 
        if (diff<-180) : diff = 360+diff
        
        # go straight if the difference between the two angles is less than 15
        if abs(diff) < range_angle:
            diff = 0
        motor_left_speed = int(thymio_speed + diff*turn_factor)
        motor_right_speed = int(thymio_speed - diff*turn_factor)

        return motor_right_speed, motor_left_speed
        
    elif current_plan == 1:
        obst = [prox_value[0], prox_value[1], prox_value[2], prox_value[3], prox_value[4]]
        # adjustment for obstacles
        if sum(obst)==0 and turn != -1:
            turn = turn
        else:    
            turn = ((obst[3] + obst[4]) > (obst[0] + obst[1]))

        # turn clockwise if proximity sensors value is below the threshold low
        if turn==1:
            spLeft = -50
            spRight = 50
            if max(obst[3:5]) < obstThrL:
                spLeft = 200
                spRight = 120
                # continue turning until the original path is not found
                for i in range(4,np.shape(path)[0]-saved_pos_idx):
                    if (np.abs([position_y, position_x] - path[saved_pos_idx+i]) <= np.array([1,1])).all():
                        current_plan = 0
                        turn = -1
                        current_target_idx = saved_pos_idx + i + 1
                        break

        # turn counterclockwise if proximity sensors value is below the threshold low
        if turn ==0:
            spLeft = 50
            spRight = -50
            if max(obst[0:2]) < obstThrL:
                spLeft = 120
                spRight = 200
                # continue turning until the original path is not found
                for i in range(4,np.shape(path)[0]-saved_pos_idx):
                    if (np.abs([position_y, position_x] - path[saved_pos_idx+i]) <= np.array([1,1])).all():
                        current_plan = 0
                        turn = -1
                        current_target_idx = saved_pos_idx + i + 1
                        break
                        
        if current_target_idx == len(path):
            spLeft = 0
            spRight = 0
        # motor control
        return(spRight,spLeft)