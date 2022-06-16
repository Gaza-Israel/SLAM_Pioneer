#!/usr/bin/env python
import rospy
import geometry_msgs.msg

def callback_geometry_message(data):
    rospy.loginfo(rospy.get_caller_id() + "Robot input message reaceived, xvel = %f, zrot = %f",data.linear.x,data.angular.z)
def callback_landmarks(data):
    rospy.loginfo(rospy.get_caller_id() + "Robot input message reaceived, xvel = %f, zrot = %f",data.linear.x,data.angular.z)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Kalman_Filter_node', anonymous=True)

    # rospy.Subscriber("/landmarks", String, callback)
    rospy.Subscriber("/cmd_vel",   geometry_msgs.msg.Twist, callback_geometry_message)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()