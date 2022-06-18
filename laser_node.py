#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import std_msgs
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from range_finder_features_pkg.msg import features_msg
class laser_node:
    def __init__(self):
        self.laser_features_publisher = rospy.Publisher("laser_features",features_msg,queue_size=1)

    def pub_laser_features(self,features,feature_idx):
        msg = features_msg()
        for feature_tuple in zip(features,feature_idx):
            p = geometry_msgs.msg.Point()
            p.x = feature_tuple[0][0]
            p.y = feature_tuple[0][1]
            p.z = feature_tuple[1]
            self.msg.points.append(p)
        self.laser_features_publisher.publish(msg)


    def laser_scan_callback(self,data,pointer):
        features = []
        features_idx = []
        # for idx,point in enumerate(data.points):
        #     features[idx,:] = [point.x,point.y]
        #     features_idx[idx] = point.z
        #
        #       KALMAN FILTER CODE
        #
        pointer.points = data.points
    
    def Map_features_callback(self,data):
        pass
    def EKF_pose_callback(self,data):
        pass


    def loop(self):
        last_input = geometry_msgs.msg.Twist()
        last_features = features_msg()
        rospy.init_node('Laser_node', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)
        rospy.Subscriber("EKF/Landmarks", Marker, self.Map_features_callback)
        rospy.Subscriber("EKF/pose", PoseWithCovarianceStamped, self.EKF_pose_callback)
        r = rospy.Rate(1) # 10hz 
        while not rospy.is_shutdown():
            #
            #       KALMAN FILTER CODE
            #
            self.pub_estimated_pose([0,0,0],np.zeros((9)))
            land = np.array([[1,2],[2,1],[1,1],[1.5,1.5]])
            self.pub_map_landmarks(land)


            r.sleep()

if __name__ == '__main__':
    ekf = EKF_node()
    ekf.loop()