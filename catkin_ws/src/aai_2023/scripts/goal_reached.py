#!/usr/bin/env python3
"""
This node determines if the car has reached the goal distance 
"""
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D

def callback_curr_pose(msg):
    global pub_goal_reached, goal_reached, goal_dist   

    x = msg.x
    y = msg.y
    theta = msg.theta
 
    goal_reached = False
    if x > goal_dist:
       goal_reached =  True      

    pub_goal_reached.publish(goal_reached)


def main():
    global pub_goal_reached, goal_reached, goal_dist
    
    x = 0.0
    y = 0.0
    theta = 0.0
    goal_reached = False
    goal_dist = 400
    
    print("INITIALIZING GOAL_REACHED NODE...", flush=True)
    rospy.init_node("goal_reached")
    rate = rospy.Rate(10)
    
    if rospy.has_param('~goal_dist'):
        goal_dist = rospy.get_param('~goal_dist')
            

    rospy.Subscriber("/self_driving_pose", Pose2D, callback_curr_pose)
    
    pub_goal_reached  = rospy.Publisher("/goal_reached", Bool, queue_size=1)

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

