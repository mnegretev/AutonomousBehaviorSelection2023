#!/usr/bin/env python3
"""
This node determines if the car is at the right or left lane of the road. 
"""
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D

def callback_curr_pose(msg):
    global pub_right_lane, right_lane, x, y, theta # , prev_right_lane   

    x = msg.x
    y = msg.y
    theta = msg.theta
 
    if y > 0:
       right_lane =  False # False indicates the car is in the left lane
    else:
       right_lane = True  # True indicates the car is in the right lane     

    pub_right_lane.publish(right_lane)


def main():
    global pub_right_lane, right_lane, x, y, theta
    
    x = 0.0
    y = 0.0
    theta = 0.0
    right_lane = True
    
    print("INITIALIZING LANE IDENTIFICATION NODE...", flush=True)
    rospy.init_node("lane_identification")
    rate = rospy.Rate(10)

    rospy.Subscriber("/self_driving_pose", Pose2D, callback_curr_pose)
    
    pub_right_lane  = rospy.Publisher("/current_lane", Bool, queue_size=1)

    while not rospy.is_shutdown():
        rate.sleep()
   

if __name__ == "__main__":
    main()

'''
if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass
'''    

