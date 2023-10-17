#!/usr/bin/env python3
"""
This node implements the action policy of the self-driving car
obtained from modeling and training MDPs using MDP-ProbLog.
This node provides several functions to check if there are
other vehicles around the car and to execute the three
different behaviors: cruise, follow and change_lane. 
"""
import rospy
from std_msgs.msg import Float64MultiArray, Empty, Bool, String, Float64
from rosgraph_msgs.msg import Clock 
import sys

def mysleep(secs):
    global curr_time

    init_time = curr_time        
    diff = 0.0
    while diff <= secs: # and not rospy.is_shutdown():
       diff  = curr_time - init_time


def callback_sim_time(msg):
    global curr_time            
    sim_time = msg
    sim_secs = sim_time.clock.secs 
    sim_nsecs = sim_time.clock.nsecs
    curr_time = sim_secs + sim_nsecs / (10**9) 

def callback_free_N(msg):
    global free_N
    free_N = msg.data

def callback_free_NW(msg):
    global free_NW
    free_NW = msg.data

def callback_free_W(msg):
    global free_W
    free_W = msg.data
    
def callback_free_SW(msg):
    global free_SW
    free_SW = msg.data
    
def callback_free_NE(msg):
    global free_NE
    free_NE = msg.data    

def callback_free_E(msg):
    global free_E
    free_E = msg.data    

def callback_free_SE(msg):
    global free_SE
    free_SE = msg.data    
    
def callback_success(msg):
    global success
    success = msg.data
    
def callback_curr_lane(msg):
    global curr_lane
    curr_lane = msg.data
    
def callback_change_lane_finished(msg):
    global change_lane_finished
    change_lane_finished = msg.data    

def cruise():
    global pub_keep_distance, pub_cruise, pub_change_lane_on_left, pub_change_lane_on_right, pub_action    
    pub_keep_distance.publish(False)
    pub_change_lane_on_left.publish(False)
    pub_change_lane_on_right.publish(False)    
    pub_cruise.publish(True)    

def keep_distance():
    global pub_keep_distance, pub_cruise, pub_change_lane_on_left, pub_change_lane_on_right, pub_action, curr_lane
    pub_cruise.publish(False)
    pub_change_lane_on_left.publish(False)
    pub_change_lane_on_right.publish(False)    
    pub_keep_distance.publish(True)    

def change_lane_on_left():
    global pub_keep_distance, pub_cruise, pub_change_lane_on_left, pub_action, curr_lane
    global curr_time

    pub_keep_distance.publish(False)
    pub_cruise.publish(False)    
    pub_change_lane_on_right.publish(False)
    pub_change_lane_on_left.publish(True)
    
def change_lane_on_right():
    global pub_keep_distance, pub_cruise, pub_change_lane_on_right, pub_action, curr_lane
    global curr_time

    pub_keep_distance.publish(False)
    pub_cruise.publish(False)    
    pub_change_lane_on_left.publish(False)    
    pub_change_lane_on_right.publish(True)    
            
def main():
    global free_N, free_NW, free_W, free_SW, free_NE, free_E, free_SE,  success, curr_lane, change_lane_finished
    global pub_keep_distance, pub_cruise, pub_change_lane_on_left, pub_change_lane_on_right, pub_action
    global curr_time
    
    curr_time = 0.0    
    
    print("INITIALIZING POLICY...", flush=True)
    rospy.init_node("policy")
    rate = rospy.Rate(10) #Hz  
    
    if rospy.has_param('vel_cars_left_lane'):
        max_speed = rospy.get_param('vel_cars_left_lane')    
        print("vel_cars_left_lane", vel_cars_left_lane)
    if rospy.has_param('vel_cars_right_lane'):
        max_speed = rospy.get_param('vel_cars_right_lane')    
        print("vel_cars_right_lane", vel_cars_right_lane)
        
    rospy.Subscriber("/clock", Clock, callback_sim_time)    
    rospy.Subscriber("/free/north"     , Bool, callback_free_N)
    rospy.Subscriber("/free/north_west", Bool, callback_free_NW)
    rospy.Subscriber("/free/west"      , Bool, callback_free_W)
    rospy.Subscriber("/free/south_west", Bool, callback_free_SW)
    rospy.Subscriber("/free/north_east"      , Bool, callback_free_NE)        
    rospy.Subscriber("/free/east"      , Bool, callback_free_E)    
    rospy.Subscriber("/free/south_east"      , Bool, callback_free_SE)
    rospy.Subscriber("/success", Bool, callback_success) 
    rospy.Subscriber("/current_lane", Bool, callback_curr_lane)
    #rospy.Subscriber("/change_lane_finished", Bool, callback_change_lane_finished)    
       
    pub_policy_started  = rospy.Publisher("/policy_started", Empty, queue_size=1)
    pub_cruise = rospy.Publisher("/cruise/enable", Bool, queue_size=1)
    pub_keep_distance    = rospy.Publisher("/follow/enable", Bool, queue_size=1)
    pub_change_lane_on_left = rospy.Publisher("/start_change_lane_on_left", Bool, queue_size=1)
    pub_change_lane_on_right = rospy.Publisher("/start_change_lane_on_right", Bool, queue_size=1)    
    pub_action = rospy.Publisher("/action", String, queue_size=1)
    
    pub_speed_cars_left_lane = rospy.Publisher("/speed_cars_left_lane", Float64, queue_size=1)
    pub_speed_cars_right_lane = rospy.Publisher("/speed_cars_right_lane", Float64, queue_size=1)    

    free_N  = True
    free_NW = True
    free_W  = True
    free_SW = True
    free_NE = True
    free_E  = True                
    free_SE = True
    success = True
    curr_lane = True

    # Pause policy.py a little bit
    rate = rospy.Rate(1) #Hz
    i = 0
    while not rospy.is_shutdown() and i < 2:
        pub_policy_started.publish()
        rate.sleep()
        print("Publishing policy_started", i )
        i = i + 1
    rate = rospy.Rate(10) #Hz

    # Move all the obstacle vehicles
    speed_cars_left_lane  = rospy.get_param("~left_cars_speed" , 5.0)
    speed_cars_right_lane = rospy.get_param("~right_cars_speed", 5.0)

    action = action_prev = "NA"            
    while not rospy.is_shutdown():
        pub_policy_started.publish()
        pub_speed_cars_left_lane.publish(speed_cars_left_lane)
        pub_speed_cars_right_lane.publish(speed_cars_right_lane)    
         
        #print("Current lane", curr_lane, "Current time", curr_time, flush = True)
        
        # right lane
        if curr_lane:
           
              if not free_N and not free_NW and not free_W and not free_SW:
                 action = "Keep distance 1"
                 pub_action.publish(action)                 
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 keep_distance()

              elif free_N and not free_NW and not free_W and not free_SW:
                 action = "Cruise 2"
                 pub_action.publish(action)                  
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 cruise()

              elif not free_N and free_NW and not free_W and not free_SW:
                 action = "Keep distance 3"
                 pub_action.publish(action)                  
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 keep_distance()

              elif free_N and free_NW and not free_W and not free_SW:
                 action = "Cruise 4"
                 pub_action.publish(action)                  
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 cruise()

              elif not free_N and not free_NW and free_W and not free_SW:
                 action = "Keep distance 5"
                 pub_action.publish(action) 
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 keep_distance()

              elif free_N and not free_NW and free_W and not free_SW:
                 action = "Cruise 6"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)               
                 cruise()

              elif not free_N and free_NW and free_W and not free_SW:
                 action = "Change lane 7"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action 
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                                   
                 change_lane_on_left()

                 print ("Waiting for change lane to finish...", flush = True, end="")
                 rospy.wait_for_message("/change_lane_finished", Bool, timeout=10000.0)
                 print (" End", flush = True)
                 
              elif free_N and free_NW and free_W and not free_SW:
                 action = "Cruise 8"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 cruise()

              elif not free_N and not free_NW and not free_W and free_SW:
                 action = "Keep distance 9"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 keep_distance()

              elif free_N and not free_NW and not free_W and free_SW:
                 action = "Cruise 10"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 cruise()

              elif not free_N and free_NW and not free_W and free_SW:
                 action = "Keep distance 11"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 keep_distance()
                 
              elif free_N and free_NW and not free_W and free_SW:
                 action = "Cruise 12"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 cruise()

              elif not free_N and not free_NW and free_W and free_SW:
                 action = "Keep distance 13"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 keep_distance()

              elif free_N and not free_NW and free_W and free_SW:
                 action = "Cruise 14"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)                
                 cruise()

              elif not free_N and free_NW and free_W and free_SW:
                 action = "Change lane 15"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)               
                 change_lane_on_left()
                 print ("Waiting for change lane to finish...", flush = True, end="")
                 rospy.wait_for_message("/change_lane_finished", Bool, timeout=10000.0)
                 print (" End", flush = True)

              elif free_N and free_NW and free_W and free_SW:
                 action = "Cruise 16"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print("free_N", free_N, "free_NW", free_NW, "free_SW", free_SW, "free_W", free_W,  flush = True)               
                 cruise()
                 
        # left lane      
        elif not curr_lane:

              if not free_E and not free_N and not free_NE and not free_SE:
                 action = "Keep distance 17"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                
                 keep_distance()

              elif free_E and not free_N and not free_NE and not free_SE:
                 action = "Keep distance 18"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                
                 keep_distance()

              elif not free_E and free_N and not free_NE and not free_SE:
                 action = "Cruise 19"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                
                 cruise()

              elif free_E and free_N and not free_NE and not free_SE:
                 action = "Cruise 20"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                
                 cruise()

              elif not free_E and not free_N and free_NE and not free_SE:
                 action = "Keep distance 21"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                   
                 keep_distance()

              elif free_E and not free_N and free_NE and not free_SE:
                 action = "Change lane 22"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action      
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                 
                 change_lane_on_right()
                 print ("Waiting for change lane to finish...", flush = True, end="")
                 rospy.wait_for_message("/change_lane_finished", Bool, timeout=10000.0)
                 print (" End", flush = True)

              elif not free_E and free_N and free_NE and not free_SE:
                 action = "Cruise 23"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)           
                 cruise()

              elif free_E and free_N and free_NE and not free_SE:
                 action = "Change lane 24"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action                
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                 
                 change_lane_on_right()
                 print ("Waiting for change lane to finish...", flush = True, end="")
                 rospy.wait_for_message("/change_lane_finished", Bool, timeout=10000.0)
                 print (" End", flush = True)
                 
              elif not free_E and not free_N and not free_NE and free_SE:
                 action = "Keep distance 25"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                
                 keep_distance()

              elif free_E and not free_N and not free_NE and free_SE:
                 action = "Keep distance 26"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                
                 keep_distance()

              elif not free_E and free_N and not free_NE and free_SE:
                 action = "Cruise 27"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                
                 cruise()

              elif free_E and free_N and not free_NE and free_SE:
                 action = "Cruise 28"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)               
                 cruise()

              elif not free_E and not free_N and free_NE and free_SE:
                 action = "Keep distance 29"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)               
                 keep_distance()

              elif free_E and not free_N and free_NE and free_SE:
                 action = "Change lane 30"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action                
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                 
                 change_lane_on_right()
                 print ("Waiting for change lane to finish...", flush = True, end="")
                 rospy.wait_for_message("/change_lane_finished", Bool, timeout=10000.0)
                 print (" End", flush = True)

              elif not free_E and free_N and free_NE and free_SE:
                 action = "Cruise 31"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                
                 cruise()

              elif free_E and free_N and free_NE and free_SE:
                 action = "Change lane 32"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, flush = True)
                    action_prev = action                
                    print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)
                 change_lane_on_right()
                 print ("Waiting for change lane to finish...", flush = True, end="")
                 rospy.wait_for_message("/change_lane_finished", Bool, timeout=10000.0)
                 print(" End", flush = True)
                                              
        rate.sleep()

if __name__ == "__main__":

    # if len(sys.argv) != 3:
    #     print("Usage: rosrun icra2024 policy.py speed_left speed_right (m/s)", len(sys.argv))
    # else:     
    try:
        main()
    except:
        rospy.ROSInterruptException
    pass

    

