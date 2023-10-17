#!/usr/bin/env python3
"""
This node implements change lane from right to left.
"""
import rospy
from std_msgs.msg import Float64, Empty, Bool

from rosgraph_msgs.msg import Clock 
from geometry_msgs.msg import Pose2D

def mysleep(secs):
    global curr_time

    init_time = curr_time        
    diff = 0.0
    while diff <= secs: 
       diff  = curr_time - init_time

def callback_current_pose(msg):
    global x, y, theta   

    x = msg.x
    y = msg.y
    z = msg.theta
 
def callback_curr_lane(msg):
    global curr_lane
    curr_lane = msg.data  

def callback_start(msg):
    global start
    start = msg.data
        
def callback_sim_time(msg):
    global sim_secs, sim_nsecs, curr_time            
    sim_time = msg
    sim_secs = sim_time.clock.secs 
    sim_nsecs = sim_time.clock.nsecs
    curr_time = sim_secs + sim_nsecs / (10**9)

    
def main():
    global start, curr_lane, pub_finish, x, y , theta, curr_time    
    
    print('INITIALIZING CHANGE_LANE NODE...', flush=True)
    rospy.init_node('change_lane')

    rospy.Subscriber("/change_lane/start", Bool, callback_start)
    rospy.Subscriber("/clock", Clock, callback_sim_time)
    
    rospy.Subscriber("/current_pose", Pose2D, callback_current_pose) 
    rospy.Subscriber("/current_lane", Bool, callback_curr_lane)       
        
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=1)
    pub_angle  = rospy.Publisher('/steering', Float64, queue_size=1)
    pub_finish = rospy.Publisher('/change_lane/finished', Empty, queue_size=1)
     
    rate = rospy.Rate(10)

    start = False
    curr_lane = False    
    curr_time = 0.0
    elapsed_time = 0.0
    starting_time = 0.0
    starting_lane = True
    max_time = 0.20
    from_right_to_left = True    
    while not rospy.is_shutdown():
              
        if start:
        
            print("change_lane: start change_lane ", "curr_time", curr_time, flush=True) 
            starting_lane = curr_lane
            pub_speed.publish(2.0)
            mysleep(0.1)   
            starting_time = curr_time
            
            if starting_lane:    
               max_time = 0.5
            else:
               max_time = 0.5
                               
            elapsed_time = 0
            while not (starting_lane^curr_lane) and elapsed_time < max_time:
                                 
               if starting_lane:    
                  pub_angle.publish(0.2)
                  from_right_to_left = True  
               else:
                  pub_angle.publish(-0.2)   
                  from_right_to_left = False      
               
               elapsed_time = curr_time - starting_time
               #print("elapsed_time", elapsed_time, "max_time", max_time, "starting_lane", starting_lane, "curr_lane", curr_lane)
                              
               mysleep(0.01)                    

            #print("Comienza a reorientarse curr_time", curr_time, "curr_lane", curr_lane)               
            if from_right_to_left:
               #print("Gira ruedas a la derecha")
               pub_angle.publish(-0.2)
               mysleep(0.2)
            else:
               #print("Gira ruedas a la izquierda")               
               pub_angle.publish(0.2)
               mysleep(0.2)
            #print("Termina de reorientarse curr_time", curr_time)                  
            print("change_lane: finish change_lane ", "curr_time", curr_time, "elapsed time", elapsed_time, flush=True)
                    
            pub_finish.publish()                
            start = False
                              
        else:
            continue

        rate.sleep()            
        
   

if __name__ == "__main__":
    main()

    

