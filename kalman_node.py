#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import std_msgs
import numpy as np
from time import sleep
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker 
from range_finder_features_pkg.msg import features_msg
from EKF_v3 import modelo,EKF
class EKF_node:
    def __init__(self,n_of_landmarks,sigma0,Q,dt):
        self.n_of_landmarks = n_of_landmarks
        self.x0 = np.zeros(n_of_landmarks*2 + 3)
        self.x0[0] =1.463
        self.x0[1] = -3.1
        self.x0[2] = euler_from_quaternion((0,0,0.14780941112961038,-0.9890158633619168))[2]
        

        # self.x0[0] = 4.305
        # self.x0[1] = -4.547
        # self.x0[2] = euler_from_quaternion((0,0,0.788010753606722,-0.6156614753256583))[2]


        self.sigma0 = sigma0 
        self.Q = Q 
        self.dt = dt 

        self.model = modelo(self.x0, self.dt, self.sigma0)
        self.ekf = EKF()



        self.pose_publisher = rospy.Publisher("EKF/pose",PoseWithCovarianceStamped,queue_size=1)
        self.landmarks_publisher = rospy.Publisher("EKF/Landmarks",Marker,queue_size=1)
        
        self.landmarks = Marker()
        self.landmarks.header.frame_id = "odom"
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

        quaternion = quaternion_from_euler(0, 0, state[2])

        output.pose.pose.orientation.x = quaternion[0]
        output.pose.pose.orientation.y = quaternion[1]
        output.pose.pose.orientation.z = quaternion[2]
        output.pose.pose.orientation.w = quaternion[3]

        new_cov = self.default_cov
        #print(new_cov)
        new_cov[0] = cov[0,0]
        new_cov[1] = cov[0,1]
        new_cov[5] = cov[0,2]
        new_cov[6] = cov[1,0]
        new_cov[7] = cov[1,1]
        new_cov[11] = cov[1,2]
        new_cov[30] = cov[2,0]
        new_cov[31] = cov[2,1]
        new_cov[35] = cov[2,2]
        output.pose.covariance = new_cov.flatten()

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'odom'
        output.header = h
        return output

    def pub_estimated_pose(self,state,cov):
        print(state[0:3])
        pose = self.state2pose(state,cov)
        self.pose_publisher.publish(pose)

    def pub_map_landmarks(self,x,sigma):
        landmarks = np.array(x[3:])
        landmarks = np.reshape(landmarks,(-1,2))
        self.landmarks.points.clear()
        self.landmarks.header.stamp = rospy.get_rostime()
        for idx in range(landmarks.shape[0]):
            if idx<self.ekf.conta_landmarks:
                p = geometry_msgs.msg.Point()
                p.x = landmarks[idx,0]
                p.y = landmarks[idx,1]
                p.z = 0
                self.landmarks.points.append(p)
        self.landmarks_publisher.publish(self.landmarks)

        

    def callback_geometry_message(self,data,pointer):
        pointer.angular.x = data.angular.x
        pointer.angular.y = data.angular.y
        pointer.angular.z = data.angular.z
        pointer.linear.x = data.linear.x
        pointer.linear.y = data.linear.y
        pointer.linear.z = data.linear.z

    def callback_landmarks(self,data):
        features = np.empty((len(data.points),2))
        idx_features = np.array([[]])
        for idx,point in enumerate(data.points):
            features[idx,:] = [point.x,point.y]
            idx_features = np.append(idx_features,point.z)
        self.ekf.correct_prediction(self.Q,self.model,features,idx_features)
        self.pub_estimated_pose(self.model.x,self.model.sigma)
        self.pub_map_landmarks(self.model.x,self.model.sigma)
        # pass


    def loop(self):
        last_input = geometry_msgs.msg.Twist()
        rospy.init_node('Kalman_Filter_node', anonymous=True)
        rospy.Subscriber("/cmd_vel",   geometry_msgs.msg.Twist, self.callback_geometry_message,last_input)
        rospy.Subscriber("/laser_features",   features_msg, self.callback_landmarks)
        r = rospy.Rate(0.1*1/self.dt) # 10hz 
        sleep(0.5)
        while not rospy.is_shutdown():
            self.model.move((last_input.linear.x,last_input.angular.z))

            self.pub_estimated_pose(self.model.x,self.model.sigma)
            self.pub_map_landmarks(self.model.x,self.model.sigma)

            r.sleep()

if __name__ == '__main__':
    n = 300

    # Inicializar as incertezas 
    infinito = 1e15
    sigma0 = np.identity(n*2 + 3) * infinito    
    sigma0[0][0] = 0
    sigma0[1][1] = 0
    sigma0[2][2] = 0

    Q = np.identity(2) ########## uncertanty in the measurement, bearing and range 
    Q[1][1] = 2e+2
    Q[0][0] = 2e+2
    dt = 0.1
    ekf = EKF_node(n_of_landmarks = n,sigma0 = sigma0,Q = Q,dt = dt)
    ekf.loop()
    