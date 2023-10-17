#!/usr/bin/env python3
"""
This node implements several behaviors:
* Lane tracking using a proportional control (inteded to be used together with the lane_detector node)
* Car following using also a proportional control (to be used together with the obstacle detector node)
* Change lane, using finite state machine and assuming we know the car position
* Pass, same conditions as the change lane behavior.  
"""
import cv2
import numpy
import rospy
import math
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool
from geometry_msgs.msg import Pose2D

SM_INIT = 0
SM_WAITING_FOR_NEW_TASK = 10
SM_START_STEADY_MOTION = 20
SM_START_CAR_FOLLOWING = 30
SM_TURNING_LEFT_1 = 40
SM_TURNING_LEFT_2 = 45
SM_TURNING_RIGHT_1 = 50
SM_TURNING_RIGHT_2 = 55
SM_TURNING_FINISHED = 60
SM_PASS_ON_RIGHT_1 = 70
SM_PASS_ON_RIGHT_2 = 80
SM_PASS_ON_RIGHT_3 = 90
SM_PASS_ON_RIGHT_4 = 100
SM_PASS_ON_RIGHT_5 = 110
SM_PASS_ON_LEFT_1 = 120
SM_PASS_ON_LEFT_2 = 130
SM_PASS_ON_LEFT_3 = 140
SM_PASS_ON_LEFT_4 = 150
SM_PASS_ON_LEFT_5 = 160
MAX_STEERING = 0.5
 
#
# Steering is calculated proportional to two errors: distance error and angle error.
# These errors correspond to differences between an observed line (in normal form)
# and a desired line.
# Speed is calculated as the max speed minus a speed proportional to the steering.
# In this way, car goes with lower speed in curves and at max speed in straight roads. 
#
def calculate_control(rho_l, theta_l, rho_r, theta_r, goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r, dist=None):
    global max_speed, k_rho, k_theta, k_following, dist_to_car
    error_rho_l   = goal_rho_l   - rho_l
    error_theta_l = goal_theta_l - theta_l
    error_rho_r   = rho_r   - goal_rho_r
    error_theta_r = theta_r - goal_theta_r
    if rho_l != 0 and rho_r != 0:
        error_rho   = (error_rho_l + error_rho_r)/2
        error_theta = (error_theta_l + error_theta_r)/2
    elif rho_l != 0:
        error_rho   = error_rho_l
        error_theta = error_theta_l
    else:
        error_rho   = error_rho_r
        error_theta = error_theta_r
    
    steering = -k_rho*error_rho - k_theta*error_theta
    if dist is None:
        speed = max_speed*(1 - 1.5*abs(steering))
    else:
        speed = max_speed + k_following*(dist - dist_to_car)
        if speed > max_speed:
            speed = max_speed
        if speed < -10:
            speed = -10
    return speed, steering

def callback_left_lane(msg):
    global lane_rho_l, lane_theta_l
    lane_rho_l, lane_theta_l = msg.data

def callback_right_lane(msg):
    global lane_rho_r, lane_theta_r
    lane_rho_r, lane_theta_r = msg.data

def callback_enable_cruise(msg):
    global enable_cruise, enable_follow
    enable_cruise = msg.data
    if(enable_cruise):
        enable_follow = False

def callback_enable_follow(msg):
    global enable_follow, enable_cruise
    enable_follow = msg.data
    if(enable_follow):
        enable_cruise = False

def callback_dist_to_obstacle(msg):
    global dist_to_obs
    dist_to_obs = msg.data

def callback_start_change_lane_on_left(msg):
    global start_change_lane_on_left, enable_follow, enable_cruise
    start_change_lane_on_left = msg.data
    if start_change_lane_on_left:
        enable_follow, enable_cruise = False, False
    

def callback_start_change_lane_on_right(msg):
    global start_change_lane_on_right, enable_follow, enable_cruise
    start_change_lane_on_right = msg.data
    if start_change_lane_on_right:
        enable_follow, enable_cruise = False, False

def callback_start_pass_on_left(msg):
    global start_pass_on_left, enable_follow, enable_cruise
    start_pass_on_left = msg.data
    if(start_pass_on_left):
        enable_follow, enable_cruise = False, False

def callback_start_pass_on_right(msg):
    global start_pass_on_right, enable_follow, enable_cruise
    start_pass_on_right = msg.data
    if(start_pass_on_right):
        enable_follow, enable_cruise = False, False

def callback_curret_pose(msg):
    global current_x, current_y, current_a
    current_x = msg.x
    current_y = msg.y
    current_a = msg.theta

def callback_free_north(msg):
    global free_north
    free_north = msg.data
    
def callback_free_north_west(msg):
    global free_north_west
    free_north_west = msg.data
    
def callback_free_west(msg):
    global free_west
    free_west = msg.data
    
def callback_free_south_west(msg):
    global free_south_west
    free_south_west = msg.data
    
def callback_free_north_east(msg):
    global free_north_east
    free_north_east = msg.data
    
def callback_free_east(msg):
    global free_east
    free_east = msg.data
    
def callback_free_south_east(msg):
    global free_south_east
    free_south_east = msg.data    

def calculate_turning_steering(w, L, v):
    # Steering is calculated from the kinematic model:  w = (v sin(d)) / L where:
    # v = current vehicle speed
    # w = desired angular speed
    # L = distance axis to axis
    if v == 0:
        return 0
    k = L * w / v

    if k > 0.5:
        k = 0.5
    if k < -0.5:
        k = -0.5

    steering = math.asin(k)
    if steering > MAX_STEERING: 
       print("Setting steering to ", MAX_STEERING)
       steering = MAX_STEERING
    elif steering < -MAX_STEERING:
       print("Setting steering to ", MAX_STEERING)
       steering = -MAX_STEERING       
 
    return steering
    
def main():
    global free_north, free_north_west, free_west, free_south_west, free_north_east, free_east, free_south_east
    global lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r, current_x, current_y, current_a
    global max_speed, k_rho, k_theta, k_following, dist_to_car
    global enable_cruise, enable_follow, dist_to_obs, start_change_lane_on_left, start_change_lane_on_right
    global start_pass_on_left, start_pass_on_right
    
    max_speed = 30      #Maximum speed for following and steady motion behaviors
    k_rho   = 0.001     #Gain for rho error in lane tracking
    k_theta = 0.01      #Gain for theta error in lane tracking
    k_following = 10.0
    dist_to_car = 30
    lane_rho_l   = 0
    lane_theta_l = 0
    lane_rho_r   = 0
    lane_theta_r = 0
    goal_rho_l   = 370.0
    goal_theta_l = 2.4
    goal_rho_r   = 430.0
    goal_theta_r = 0.895
    
    print('INITIALIZING BEHAVIORS NODE...', flush=True)
    rospy.init_node('lane_tracking')
    rate = rospy.Rate(10)
    if rospy.has_param('~max_speed'):
        max_speed = rospy.get_param('~max_speed')
    if rospy.has_param('~k_rho'):
        k_rho = rospy.get_param('~k_rho')
    if rospy.has_param('~k_theta'):
        k_theta = rospy.get_param('k_theta')
    if rospy.has_param('~k_following'):
        k_following = rospy.get_param('~k_following')
    if rospy.has_param('~dist_to_car'):
        dist_to_car = rospy.get_param('~dist_to_car')

    print("Waiting for lane detection...")
    rospy.Subscriber("/demo/left_lane" , Float64MultiArray, callback_left_lane)
    rospy.Subscriber("/demo/right_lane", Float64MultiArray, callback_right_lane)
    rospy.Subscriber("/cruise/enable", Bool, callback_enable_cruise)
    rospy.Subscriber("/follow/enable", Bool, callback_enable_follow)
    rospy.Subscriber("/start_change_lane_on_left", Bool, callback_start_change_lane_on_left)
    rospy.Subscriber("/start_change_lane_on_right", Bool, callback_start_change_lane_on_right)
    rospy.Subscriber("/start_pass_on_left" , Bool, callback_start_pass_on_left)
    rospy.Subscriber("/start_pass_on_right", Bool, callback_start_pass_on_right)
    rospy.Subscriber("/obstacle/distance", Float64, callback_dist_to_obstacle)
    rospy.Subscriber("/self_driving_pose", Pose2D, callback_curret_pose)
    rospy.Subscriber("/free/north"     , Bool, callback_free_north)
    rospy.Subscriber("/free/north_west", Bool, callback_free_north_west)
    rospy.Subscriber("/free/west"      , Bool, callback_free_west)
    rospy.Subscriber("/free/south_west", Bool, callback_free_south_west)
    rospy.Subscriber("/free/north_east", Bool, callback_free_north_east) 
    rospy.Subscriber("/free/east"      , Bool, callback_free_east)      
    rospy.Subscriber("/free/south_east", Bool, callback_free_south_east)
    
    pub_speed = rospy.Publisher('/speed', Float64, queue_size=1)
    pub_angle = rospy.Publisher('/steering', Float64, queue_size=1)
    pub_change_lane_finshed = rospy.Publisher('/change_lane_finished', Bool, queue_size=1)
    pub_pass_finished = rospy.Publisher('/pass_finished', Bool, queue_size=1)
    msg_left_lane  = rospy.wait_for_message('/demo/left_lane' , Float64MultiArray, timeout=100)
    msg_right_lane = rospy.wait_for_message('/demo/right_lane', Float64MultiArray, timeout=100)
    print("Using:")
    print("Max speed: " + str(max_speed))
    print("K_rho: " + str(k_rho))
    print("K_theta: " + str(k_theta))
    print("K_following: " + str(k_following))
    print("Dist to car: " + str(dist_to_car))
    enable_cruise = False
    enable_follow = False
    dist_to_ob    = None
    start_change_lane_on_left = False
    start_change_lane_on_right= False
    start_pass_on_left        = False
    start_pass_on_right       = False

    state = SM_INIT
    speed, steering = 0,0
    current_x, current_y, current_a = 0,0,0
    last_x = 0
    while not rospy.is_shutdown():
        if state == SM_INIT:
            print("Waiting for task...")
            state = SM_WAITING_FOR_NEW_TASK

        elif state == SM_WAITING_FOR_NEW_TASK:
            if enable_cruise:
                state = SM_START_STEADY_MOTION
                print("Starting steady motion")
            elif enable_follow:
                state = SM_START_CAR_FOLLOWING
                print("Starting follow car")
            elif start_change_lane_on_left:
                state = SM_TURNING_LEFT_1
                start_change_lane_on_left = False
                print("Starting change lane on left")
            elif start_change_lane_on_right:
                state = SM_TURNING_RIGHT_1
                start_change_lane_on_right = False
                print("Starting change lane on right")
            elif start_pass_on_left:
                state = SM_PASS_ON_LEFT_1
                start_pass_on_left = False
                print("Starting passing on left")
            elif start_pass_on_right:
                state = SM_PASS_ON_RIGHT_1
                start_pass_on_right = False
                print("Starting passing on right")
            else:
                speed, steering = 0,0

        elif state == SM_START_STEADY_MOTION:
            speed, steering = calculate_control(lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r, goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r)
            dist_to_obs = None
            
            if not enable_cruise:
                state = SM_WAITING_FOR_NEW_TASK

        elif state == SM_START_CAR_FOLLOWING:
            speed,steering = calculate_control(lane_rho_l,lane_theta_l,lane_rho_r,lane_theta_r,goal_rho_l,goal_theta_l,goal_rho_r,goal_theta_r,dist_to_obs)
            if not enable_follow:
                state = SM_WAITING_FOR_NEW_TASK


        #
        # STATES FOR CHANGE TO LEFT LANE
        #
        elif state == SM_TURNING_LEFT_1:
            if speed <= 10:
                speed = max_speed
            steering = calculate_turning_steering(1.2, 2.9, speed)
            if current_y > -0.7: # Vehicle has moved to the left. Right lane has y=-1.5 and center is around y=0
                print("Moving to right to align with left lane")
                state = SM_TURNING_LEFT_2 

        elif state == SM_TURNING_LEFT_2:
            if speed <= 10:
                speed = max_speed
            steering = calculate_turning_steering(-1.2, 2.9, speed)
            if current_y > 1.0 or abs(current_a) < 0.2: # Vehicle has moved to the left lane. Left lane has y=1.5
                print("Change lane on left finished")
                pub_change_lane_finshed.publish(True)
                state = SM_WAITING_FOR_NEW_TASK

        #
        # STATES FOR CHANGE TO RIGHT LANE
        #
        elif state == SM_TURNING_RIGHT_1:
            if speed <=10:
                speed = max_speed
            steering = calculate_turning_steering(-1.2, 2.9, speed)
            if current_y < 0.7: #Vehicle has moved to the right. Left lane has y = 1.5 and center is around y=0
                print("Moving to left to align with right lane")
                state = SM_TURNING_RIGHT_2

        elif state == SM_TURNING_RIGHT_2:
            if speed <= 10:
                speed = max_speed
            steering = calculate_turning_steering(1.2, 2.9, speed)
            if current_y < -1.0 or abs(current_a) < 0.2: #Vehicle has moved to the right lane. Right lane has y=-1.5
                print("Change lane on right finished")
                pub_change_lane_finshed.publish(True)
                state = SM_WAITING_FOR_NEW_TASK


        #
        # STATES FOR PASSING ON THE LEFT
        #
        elif state == SM_PASS_ON_LEFT_1:
            if speed <= 10:
                speed = max_speed
            steering = calculate_turning_steering(1.2, 2.9, speed)
            if current_y > -0.7: # Vehicle has moved to the left. Right lane has y=-1.5 and center is around y=0
                print("Moving to right to align with left lane")
                state = SM_PASS_ON_LEFT_2

        elif state == SM_PASS_ON_LEFT_2:
            if speed <= 10:
                speed = max_speed
            steering = calculate_turning_steering(-1.2, 2.9, speed)
            if current_y > 1.0 or abs(current_a) < 0.2: # Vehicle has moved to the left lane. Left lane has y=1.5
                print("Moving to left lane finished. Starting to follow lane")
                state = SM_PASS_ON_LEFT_3
                last_x = current_x

        elif state == SM_PASS_ON_LEFT_3:
            speed, steering = calculate_control(lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r, goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r)
            if free_north_east and free_east:
                state = SM_PASS_ON_LEFT_4

        elif state == SM_PASS_ON_LEFT_4:
            if speed == 0:
                speed = max_speed
            steering = calculate_turning_steering(-1.2, 2.9, speed)
            if current_y < 0.7: #Vehicle has moved to the right. Left lane has y = 1.5 and center is around y=0
                print("Moving to right to align with right lane after passing on left")
                state = SM_PASS_ON_LEFT_5

        elif state == SM_PASS_ON_LEFT_5:
            if speed <= 10:
                speed = max_speed
            steering = calculate_turning_steering(1.2, 2.9, speed)
            if current_y < -1.0 or abs(current_a) < 0.2: #Vehicle has moved to the right lane. Right lane has y=-1.5
                pub_pass_finished.publish(True)
                print("Passing on left finished")
                state = SM_WAITING_FOR_NEW_TASK


        #
        # STATES FOR PASSING ON THE RIGHT
        #
        elif state == SM_PASS_ON_RIGHT_1:
            if speed <= 10:
                speed = max_speed
            steering = calculate_turning_steering(-1.2, 2.9, speed)
            if current_y < 0.7: # Vehicle has moved to the right Right lane has y=-1.5 and center is around y=0
                print("Moving to left to align with right lane")
                state = SM_PASS_ON_RIGHT_2

        elif state == SM_PASS_ON_RIGHT_2:
            if speed <= 10:
                speed = max_speed
            steering = calculate_turning_steering(1.2, 2.9, speed)
            if current_y < -1.0 or abs(current_a) < 0.2: # Vehicle has moved to the right lane. Right lane has y=-1.5
                print("Moving to right lane finished. Starting to follow lane.")
                state = SM_PASS_ON_RIGHT_3
                last_x = current_x

        elif state == SM_PASS_ON_RIGHT_3:
            speed, steering = calculate_control(lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r, goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r)
            if free_north_west and free_west:
                state = SM_PASS_ON_RIGHT_4

        elif state == SM_PASS_ON_RIGHT_4:
            if speed <= 10:
                speed = max_speed
            steering = calculate_turning_steering(1.2, 2.9, speed)
            if current_y > -0.7: #Vehicle has moved to the right. Left lane has y = 1.5 and center is around y=0
                print("Moving to right to align with left lane after passing on right")
                state = SM_PASS_ON_RIGHT_5

        elif state == SM_PASS_ON_RIGHT_5:
            if speed <= 10:
                speed = max_speed
            steering = calculate_turning_steering(-1.2, 2.9, speed)
            if current_y > 1.0 or abs(current_a) < 0.2: #Vehicle has moved to the left lane. Left lane has y=1.5
                print("Passing on right finished")
                pub_pass_finished.publish(True)
                state = SM_WAITING_FOR_NEW_TASK
                
        else:
            print("Invalid STATE")
            break;
        #print([speed, steering])
        pub_speed.publish(speed)
        pub_angle.publish(steering)
        
        rate.sleep()

    

if __name__ == "__main__":
    main()

    

