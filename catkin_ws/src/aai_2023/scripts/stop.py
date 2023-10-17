#!/usr/bin/env python3
"""
This node stop the car by killing the nodes that control the car and sets the car speed to 0
  
"""
import os
import rospy
from std_msgs.msg import Float64, Empty, Bool
   
def callback_success(msg):
    global success
    success = msg.data    
    
def stop_motion():
    global pub_speed

    speed = 0.0  
    pub_speed.publish(speed) 
    
def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data             

def main():

    global success, pub_speed, goal_reached
            
    print('INITIALIZING STOP NODE...', flush=True)
    rospy.init_node('stop')
    rate = rospy.Rate(10)

    success = True
    goal_reached = False

    rospy.Subscriber("/success", Bool, callback_success) 
    rospy.Subscriber("/goal_reached", Bool, callback_goal_reached)      
    pub_speed = rospy.Publisher('/speed', Float64, queue_size=1)

    while not rospy.is_shutdown():
        if not success or goal_reached:
            print("STOP: Starting system shutdown...",  flush = True)  
            
            os.system("rosnode kill /policy")
            print("STOP: Finishing system shutdown... ", end="", flush = True)

            stop_motion()
            
            print("Done", flush = True)             

            os.system("rosnode kill /behaviors")
            os.system("rosnode kill /current_lane")
            os.system("rosnode kill /lane_detector")
            os.system("rosnode kill /obstacle_detector")
            os.system("rosnode kill /success")
            os.system("rosnode kill /stop")     
                   
        else: # do_nothing
            continue

        rate.sleep()
    

if __name__ == "__main__":
    main()

    

