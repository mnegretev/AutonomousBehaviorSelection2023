#!/usr/bin/env python3
"""
This node detect obstacles around the car using the lidar sensor.
Obstacles are detected only by counting the number of points inside 
a bounding box. Each BB defines a region around de car:
----------------|---------|--------------|
North/North East| East    | South East   |
----------------|---------|--------------|
North/North-west| West    | South West   |
----------------|---------|--------------|
"""
import math
import numpy
import rospy
import ros_numpy
from std_msgs.msg import Float64MultiArray, Empty, Bool, Float64
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock 

def callback_sim_time(msg):
    global sim_secs, sim_nsecs, curr_time          
    sim_time = msg
    sim_secs = sim_time.clock.secs 
    sim_nsecs = sim_time.clock.nsecs 
    curr_time = sim_secs + sim_nsecs / (10**9)        

def callback_point_cloud(msg):
    global pub_obs_N, pub_obs_NW, pub_obs_W, pub_obs_south_west, pub_obs_NE, pub_obs_E, pub_obs_south_east, pub_obs_dist, curr_time
    
    xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    xyz = xyz[(xyz[:,2] > -1) & (xyz[:,2] < 0.3) ] #Filters points on floor and higher points
    N_points  = xyz[(xyz[:,0] >  2.5) & (xyz[:,0] <  25) & (xyz[:,1] < 1.5) & (xyz[:,1] > -1.5)]
    NW_points = xyz[(xyz[:,0] >  5) & (xyz[:,0] <   25) & (xyz[:,1] < 4.0) & (xyz[:,1] >  1.2)]
    W_points  = xyz[(xyz[:,0] > -5) & (xyz[:,0] <  5) & (xyz[:,1] < 4.0) & (xyz[:,1] >  1.2)]
    SW_points = xyz[(xyz[:,0] >  -25) & (xyz[:,0] < -5) & (xyz[:,1] < 4.0) & (xyz[:,1] >  1.2)]
    
    NE_points  = xyz[(xyz[:,0] >  5) & (xyz[:,0] <   25) & (xyz[:,1] > -4.0) & (xyz[:,1] <  -1.2)]        
    E_points  = xyz[(xyz[:,0] > -5) & (xyz[:,0] <  5) & (xyz[:,1] > -4.0) & (xyz[:,1] <  -1.2)] 
    SE_points  = xyz[(xyz[:,0] >  -25) & (xyz[:,0] < -5) & (xyz[:,1] > -4.0) & (xyz[:,1] <  -1.2)]               
    
    free_N  = N_points .shape[0] < 20
    free_NW = NW_points.shape[0] < 50
    free_W  = W_points .shape[0] < 50
    free_SW = SW_points.shape[0] <  50

    free_NE  = NE_points .shape[0] < 50    
    free_E  = E_points .shape[0] < 50   
    free_SE  = SE_points .shape[0] < 50   
    
    pub_obs_N .publish(free_N )
    pub_obs_NW.publish(free_NW)
    pub_obs_W .publish(free_W )
    pub_obs_SW.publish(free_SW)
    pub_obs_NE .publish(free_NE)    
    pub_obs_E .publish(free_E)            
    pub_obs_SE .publish(free_SE)

    #print("N", N_points.shape[0], "NW", NW_points.shape[0], "W", W_points.shape[0], "SW", SW_points .shape[0], "NE", NE_points.shape[0], "E", E_points.shape[0], "SE", SE_points .shape[0], "curr_time", curr_time, flush = True)
    obs_points  = xyz[(xyz[:,0] >  2.5) & (xyz[:,0] < 100) & (xyz[:,1] < 1.0) & (xyz[:,1] > -1.0)]
    if obs_points.shape[0] > 10:
       #print("North points", N_points)
       pub_obs_dist.publish(numpy.linalg.norm(numpy.mean(obs_points, axis=0)))
       #print("average distance north", numpy.linalg.norm(numpy.mean(N_points, axis=0)), flush = True) 

def main():
    global pub_obs_N, pub_obs_NW, pub_obs_W, pub_obs_SW, pub_obs_NE, pub_obs_E, pub_obs_SE, pub_obs_dist
    global curr_time
    
    print("INITIALIZING OBSTACLE DETECTOR...", flush=True)
    rospy.init_node("free_detector")
    rate = rospy.Rate(20)
        
    rospy.Subscriber('/point_cloud', PointCloud2, callback_point_cloud)
    rospy.Subscriber("/clock", Clock, callback_sim_time)
        
    pub_obs_N  = rospy.Publisher("/free/north"     , Bool, queue_size=1)
    pub_obs_NW = rospy.Publisher("/free/north_west", Bool, queue_size=1)
    pub_obs_W  = rospy.Publisher("/free/west"      , Bool, queue_size=1)
    pub_obs_SW = rospy.Publisher("/free/south_west", Bool, queue_size=1)
    pub_obs_NE = rospy.Publisher("/free/north_east"      , Bool, queue_size=1) 
    pub_obs_E  = rospy.Publisher("/free/east"      , Bool, queue_size=1)      
    pub_obs_SE = rospy.Publisher("/free/south_east"      , Bool, queue_size=1)
    
    pub_obs_dist = rospy.Publisher("/obstacle/distance", Float64, queue_size=1)
        
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
    

