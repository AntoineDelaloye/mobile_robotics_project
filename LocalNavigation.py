# Import tdmclient Notebook environment:
import tdmclient.notebook
import numpy as np
import PathPlanning
from ekf import ExtendedKalmanFilter
from PathPlanning import Global_navigation
import math

obstThrL = 500      # low obstacle threshold to switch state 1->0
obstThrH = 900      # high obstacle threshold to switch state 0->1
obstSpeedGain = [0.06, 0.04, 0, -0.04, -0.06]  # gain for motor speed necessary to avoid obstacle
turn = -1
spLeft = 50
spRight =50
current_target_idx = 0
current_plan = 0           # 0=A* algorithm, 1=obstacle avoidance
obst  = [0,0,0,0,0]  # measurements from left and right prox sensors
#### Need to define them directly as global vairables. Can't define them AFTER as global variables

#@tdmclient.notebook.sync_var 
def obstacleAvoidance(position_x, position_y,prox_value,path):
    """
    This function is used to detect an obstacle, not present while doing the path planning, and overcome it without colliding.
    The Thymio will continue to turn until he reaches the first target after the obstacle, from his global path planning.
    :param obst: a row vector of 5 values from the proximity sensors in front of the Thymio 
    :param kalman_position: actual postion of the robot 
    :param path: matrix composed by coordinates (x,y) of the global path planning
    :param saved_pos: index from 
    """
    global obst

    obst = [prox_value[0], prox_value[1], prox_value[2], prox_value[3], prox_value[4]]
    # store the kalman_position from path before a local obstacle is detected
    saved_pos = checkState(position_x, position_y,prox_value)
    saved_pos_idx = 0

    # store the index of the kalman_position from path before a local obstacle is detected
    saved_pos_idx = 0
    for i in range(len(path)):
        if (path[i] == saved_pos).all():
            saved_pos_idx = i        
    return saved_pos_idx

#@tdmclient.notebook.sync_var
def checkState(position_x, position_y,prox_value):
    """
    Returns the state of the Thymio which indicate whether to follow the path planning 
    from the A* algorithm or to avoid a local obstacle 
    :param kalman_position: the real time kalman_position of the Thymio  
    :param obstThrH: the value which the 5 proximity sensors exceed when they detect an object (14 cm)
    :param obstThrL: the value which indicate if the 5 proximity sensors don't detect anything 
    :return state: 0=A* algorithm, 1=obstacle avoidance
    :return saved_pos: kalman_position of the Thymio before finding an obstacle 
    """
    global obst, current_plan, obstThrH, obstThrL

    obst = [prox_value[0], prox_value[1], prox_value[2], prox_value[3], prox_value[4]]

    if current_plan == 0: 
        # switch from goal tracking to obst avoidance if obstacle detected
        if (max(obst) > obstThrH):
            current_plan = 1
            saved_pos = [position_y, position_x]                        
            return saved_pos
    elif current_plan == 1:
        # switch from obst avoidance to goal tracking if obstacle got unseen 
        if max(obst) < obstThrL:
            current_plan = 0
            saved_pos = 0
            return saved_pos
        
#@tdmclient.notebook.sync_var
def motionControl(kalman_position, path, square_size, prox_value): ## state and current_target_position should be global
    global saved_pos_idx, obstSpeedGain,current_plan, current_target_idx, spLeft,spRight, turn

    thymio_speed = 100
    range_angle = 15
    turn_factor = 3

    position_x, position_y, position_angle = kalman_position
    position_angle = position_angle*180/np.pi

    # position_x = round(position_x/square_size)
    # position_y = round(position_y/square_size)
    # checkState(position_x, position_y, prox_value)
    # print("current plan:",current_plan)
    # print("Prox values",prox_value)
    if current_plan == 0:
        saved_pos_idx = obstacleAvoidance(position_x, position_y, prox_value, path)
        if current_plan:
            return(0,0)
        if current_target_idx >= path.shape[0]:
            current_target_idx = path.shape[0]-1
            return (0,0)

        if (np.abs([position_y, position_x]-path[current_target_idx])<= np.array([1,1])).all():
            #([position_y, position_x] == path[current_target_idx]).all():
            current_target_idx += 1
            if current_target_idx >= path.shape[0]:
                current_target_idx = path.shape[0]-1
                return (0,0)

        targetAngle = math.atan2((path[current_target_idx][0] - position_y),(path[current_target_idx][1] - position_x))*180/np.pi %(360)

        diff = position_angle - targetAngle
        # print("diff before:",diff)
        if (diff>180) : diff = diff-360 
        if (diff<-180) : diff = 360+diff
        # print("diff after", diff)

        if abs(diff) < range_angle:
            diff = 0
        motor_left_speed = int(thymio_speed + diff*turn_factor)
        motor_right_speed = int(thymio_speed - diff*turn_factor)

        #spLeft, spRight = motor_left_speed, motor_right_speed
        return(motor_right_speed,motor_left_speed)
        
    elif current_plan == 1:
        obst = [prox_value[0], prox_value[1], prox_value[2], prox_value[3], prox_value[4]]
        # adjustment for obstacles
        if sum(obst)==0 and turn != -1:
            turn = turn
        else:    
            turn = ((obst[3] + obst[4]) > (obst[0] + obst[1]))
        # print("turn:",turn)
        # true if turning clockwise, false if turning counter-clockwise
        # for i in range(5):
        #     spLeft += int(prox_value[i] * obstSpeedGain[i]/10)
        #     spRight += int(prox_value[i] * obstSpeedGain[4 - i]/10)
        # turn clockwise if proximity sensors value is below the threshold low
        if turn==1:
            spLeft = -50
            spRight = 50
            if max(obst[3:5]) < obstThrL:
                print("sensors:",prox_value)
                spLeft = 200
                spRight = 120
                # saved_pos_idx = obstacleAvoidance(position_x, position_y,prox_value,path)
                # continue turning until the original path is not found
                for i in range(4,np.shape(path)[0]-saved_pos_idx):
                    #if [position_y, position_x] == [path[saved_pos_idx+i][0], path[saved_pos_idx+i][1]]:
                    if (np.abs([position_y, position_x] - path[saved_pos_idx+i]) <= np.array([1,1])).all():
                        current_plan = 0
                        turn = -1
                        current_target_idx = saved_pos_idx + i + 1
                        break
                # if [position_y, position_x] == [path[saved_pos_idx][0], path[saved_pos_idx][1]]:
                #     current_plan = 0
                # if [position_y, position_x] == [path[saved_pos_idx+1][0], path[saved_pos_idx+1][1]]:
                #     current_plan = 0
                # if [position_y, position_x] == [path[saved_pos_idx+2][0], path[saved_pos_idx+2][1]]:
                #     current_plan = 0
        # turn counterclockwise if proximity sensors value is below the threshold low
        if turn ==0:
            spLeft = 50
            spRight = -50
            if max(obst[0:2]) < obstThrL:
                # print("sensors:",prox_value)
                spLeft = 120
                spRight = 200
                # saved_pos_idx = obstacleAvoidance(position_x, position_y,prox_value,path)
                for i in range(4,np.shape(path)[0]-saved_pos_idx):
                    #if [position_y, position_x] == [path[saved_pos_idx+i][0], path[saved_pos_idx+i][1]]:
                    if (np.abs([position_y, position_x] - path[saved_pos_idx+i]) <= np.array([1,1])).all():
                        current_plan = 0
                        turn = -1
                        current_target_idx = saved_pos_idx + i + 1
                        break
                    # continue turning until the original path is not found
                # if [position_y, position_x] == [path[saved_pos_idx][0], path[saved_pos_idx][1]]:
                #     current_plan = 0
                # if [position_y, position_x] == [path[saved_pos_idx+1][0], path[saved_pos_idx+1][1]]:
                #     current_plan = 0
                # if [position_y, position_x] == [path[saved_pos_idx+2][0], path[saved_pos_idx+2][1]]:
                #     current_plan = 0
        if current_target_idx == len(path):
            spLeft = 0
            spRight = 0
        # motor control
        return(spRight,spLeft)
        #motor_left_target = spLeft
        #motor_right_target = spRight


def currentPositionIndex(kalman_position, path):
    global current_target_idx

    if kalman_position == path[current_target_idx+1]:
        current_target_idx += 1


