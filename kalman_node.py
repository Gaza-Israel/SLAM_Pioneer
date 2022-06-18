#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import std_msgs
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker 
from pyquaternion import Quaternion
from range_finder_features_pkg.msg import features_msg
class EKF_node:
    def __init__(self):
        self.pose_publisher = rospy.Publisher("EKF/pose",PoseWithCovarianceStamped,queue_size=1)
        self.landmarks_publisher = rospy.Publisher("EKF/Landmarks",Marker,queue_size=1)
        
        self.landmarks = Marker()
        self.landmarks.header.frame_id = "base_link"
        self.landmarks.ns = "Landmarks points"
        self.landmarks.action = Marker.ADD
        self.landmarks.pose.orientation.w = 1
        self.landmarks.id = 0
        self.landmarks.type = Marker.POINTS
        self.landmarks.scale.x = 0.08
        self.landmarks.scale.y = 0.08
        self.landmarks.color.g = 1
        self.landmarks.color.a = 1

        self.default_cov = np.zeros((36,))

    def state2pose(self, state, cov):
        #print(cov)
        output = PoseWithCovarianceStamped()
        output.pose.pose.position.x = state[0]
        output.pose.pose.position.y = state[1]
        output.pose.pose.position.z = 0

        quaternion = Quaternion(axis=[0, 0, 1], angle=state[2]).elements

        output.pose.pose.orientation.x = quaternion[0]
        output.pose.pose.orientation.y = quaternion[1]
        output.pose.pose.orientation.z = quaternion[2]
        output.pose.pose.orientation.w = quaternion[3]

        new_cov = self.default_cov
        #print(new_cov)
        new_cov[0] = cov[0]
        new_cov[1] = cov[1]
        new_cov[5] = cov[2]
        new_cov[6] = cov[3]
        new_cov[7] = cov[4]
        new_cov[11] = cov[5]
        new_cov[30] = cov[6]
        new_cov[31] = cov[7]
        new_cov[35] = cov[8]
        output.pose.covariance = new_cov.flatten()

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'base_link'
        output.header = h
        return output

    def pub_estimated_pose(self,state,cov):
        pose = self.state2pose(state,cov)
        self.pose_publisher.publish(pose)
        del pose

    def pub_map_landmarks(self,landmarks):
        self.landmarks.points.clear()
        self.landmarks.header.stamp = rospy.get_rostime()
        for idx in range(landmarks.shape[0]):
            p = geometry_msgs.msg.Point()
            p.x = landmarks[idx,0]
            p.y = landmarks[idx,1]
            p.z = 0
            self.landmarks.points.append(p)
        self.landmarks_publisher.publish(self.landmarks)

        

    def callback_geometry_message(self,data,pointer):
        # pointer.angular.x = data.angular.x
        # pointer.angular.y = data.angular.y
        # pointer.angular.z = data.angular.z
        # pointer.linear.x = data.linear.x
        # pointer.linear.y = data.linear.y
        # pointer.linear.z = data.linear.z
        buff = None
        data.serialize(buff)
        pointer.deserialize(buff)

    def callback_landmarks(self,data,pointer):
        features = []
        features_idx = []
        # for idx,point in enumerate(data.points):
        #     features[idx,:] = [point.x,point.y]
        #     features_idx[idx] = point.z
        #
        #       KALMAN FILTER CODE
        #
        pointer.points = data.points

    def loop(self):
        last_input = geometry_msgs.msg.Twist()
        last_features = features_msg()
        rospy.init_node('Kalman_Filter_node', anonymous=True)
        rospy.Subscriber("/cmd_vel",   geometry_msgs.msg.Twist, self.callback_geometry_message,last_input)
        rospy.Subscriber("/laser_features",   features_msg, self.callback_landmarks,last_features)
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