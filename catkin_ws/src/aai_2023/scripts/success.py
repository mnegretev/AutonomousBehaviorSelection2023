#!/usr/bin/env python3
"""
This node detects if the car succeeded its last action using Euclidean distance between points in two consecutive readings of the accelerometer sensor
"""
import math
import rospy
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Imu

def callback_accelerometer(msg):

    global success, first_time, pub_success, pub_accel_diff, x1, y1, z1   

    threshold = 200

    x2 = msg.linear_acceleration.x
    y2 = msg.linear_acceleration.y
    z2 = msg.linear_acceleration.z

    diff = 0
    if success:
       if first_time:
          first_time = False
       else:   
          diff = ((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)**0.5
          if diff > threshold:
             success = False
             #print("ERROR: Choque detectado", flush=True)
             #output = "x1= " + str(x1) + " x2= " + str(x2) + " y1= " + str(y1) + " y2= " + str(y2) + " z1= " + str(z1) + " z2= " + str(z2) + " diff= " + str(diff) + "\n"
             #print(output, flush = True)

       x1 = msg.linear_acceleration.x
       y1 = msg.linear_acceleration.y
       z1 = msg.linear_acceleration.z     

    pub_success.publish(success)
    pub_accel_diff.publish(diff)

def main():
    global success, first_time, pub_success, pub_accel_diff, x1, y1, z1 
     
    first_time = True
    success = True
    x1 = 0
    y1 = 0
    z1 = 0
    
    print("INITIALIZING SUCCESS...", flush=True)
    rospy.init_node("success")
    rate = rospy.Rate(10)
    rospy.Subscriber('/accelerometer', Imu, callback_accelerometer)
        
    pub_success  = rospy.Publisher("/success", Bool, queue_size=1)
    pub_accel_diff  = rospy.Publisher("/accelerometer_diff", Float64, queue_size=1)    
   
    #rospy.spin()
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
