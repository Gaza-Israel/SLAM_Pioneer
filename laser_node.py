#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import copy
import numpy as np
import cv2 as cv
from time import sleep
from io import BytesIO
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from range_finder_features_pkg.msg import features_msg
from Feature_detector import feature_detector, feature_matcher,polar2z
class Laser_node:
    def __init__(self):
        self.laser_features_publisher = rospy.Publisher("laser_features",features_msg,queue_size=1)
        self.fd = feature_detector(laser_max_range=5.6, res_map=0.01, acc_th=20, min_line_lenght=0.30, max_line_gap=0.30, min_dist2line_th=0.2, filter_min_points = 20,threshold_cluster=0.02,max_intersection_distance=10)
        self.fm = feature_matcher(5)

    def extract_laser_data_from_message(self,data:LaserScan):
        angle_array = np.arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        ranges = np.array(data.ranges)
        array_has_nan = np.logical_or(np.isnan(data.ranges), ranges<0.05)
        data.ranges = ranges[np.invert(array_has_nan)]
        angle_array = angle_array[np.invert(array_has_nan)]

        return (data.ranges, angle_array)
    def pub_laser_features(self,features,feature_idx):
        msg = features_msg()
        for feature_tuple in zip(features[:,0],features[:,1],feature_idx):
            p = geometry_msgs.msg.Point()
            p.x = feature_tuple[0]
            p.y = feature_tuple[1]
            p.z = feature_tuple[2]
            msg.points.append(p)
        self.laser_features_publisher.publish(msg)
        # print(msg)


    def laser_scan_callback(self,data,*args):

        global last_pose
        global last_landmarks

        # map_landmarks = args[0][0]
        # pose = args[0][1]
        map_landmarks = last_landmarks
        pose = last_pose
        orientation_q = pose.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        rho, theta = self.extract_laser_data_from_message(data)
        x, y = polar2z(rho, theta)
        map, map_points = self.fd.create_map(x, y)
        df, img = self.fd.detect_lines(map, plot=False)
        plot = True
        if plot:
            img = np.array(map * 255).astype("uint8")
            img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
            df = self.fd.check_points_in_line(map_points, df)
            df_filtered, img = self.fd.filter_segments(df, img)
            df_inter_filtered, img = self.fd.find_intersections(df_filtered, img)
            features = self.fd.inter2feature(df_inter_filtered)
            features,idx_feature,new_features = self.fm.match_features(
                features,map_landmarks,
                map_landmarks.shape[0],
                (pose.pose.pose.position.x,pose.pose.pose.position.y,yaw),img,df_inter_filtered)
        else:
            df = self.fd.check_points_in_line(map_points, df)
            df_filtered, img = self.fd.filter_segments(df, img)
            df_inter_filtered, img = self.fd.find_intersections(df_filtered, img)
            features = self.fd.inter2feature(df_inter_filtered)
            features,idx_feature,new_features = self.fm.match_features(
                features,map_landmarks,
                map_landmarks.shape[0],
                (pose.pose.pose.position.x,pose.pose.pose.position.y,yaw))
                    
        self.pub_laser_features(features,idx_feature)

        # sleep(5)


    
    def Map_features_callback(self,data:Marker,pointer):

        global last_landmarks
        pointer = np.array([])
        for point in data.points:
            pointer = np.append(pointer,[point.x,point.y])
        last_landmarks = np.reshape(np.array(pointer),(-1,2))
    def EKF_pose_callback(self,data:PoseWithCovarianceStamped,pointer:PoseWithCovarianceStamped):
        global last_pose
        buff = BytesIO()
        data.serialize(buff)
        last_pose.deserialize(buff.getvalue())


    def loop(self):
        global last_pose
        global last_landmarks
        last_pose = PoseWithCovarianceStamped()
        pose_on_scan = PoseWithCovarianceStamped()
        last_landmarks = np.array([])
        rospy.init_node('Laser_node', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback,(copy.copy(last_landmarks),copy.copy(last_pose)))
        rospy.Subscriber("EKF/Landmarks", Marker, self.Map_features_callback,last_landmarks)
        rospy.Subscriber("EKF/pose", PoseWithCovarianceStamped, self.EKF_pose_callback,last_pose)
        rospy.spin()

if __name__ == '__main__':
    Laser = Laser_node()
    Laser.loop()