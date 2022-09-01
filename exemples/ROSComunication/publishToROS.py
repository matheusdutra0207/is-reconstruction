#!/usr/bin/env python3
# license removed for brevity

# Python
import numpy as np

# ROS
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

# IS
from is_wire.core import Channel, Subscription
from is_msgs.common_pb2 import Pose

#  my own
from streamChannel import StreamChannel


def getPoseReconstruction():
    message = channel_recontruction.consume_last()
    if type(message) != bool:
        pose = message.unpack(Pose)
        x_recontruction = pose.position.x
        y_recontruction = pose.position.y
        yaw_rad_recontruction = pose.orientation.yaw
        return np.array([x_recontruction, y_recontruction, yaw_rad_recontruction])
    else:
        return message

def publishToRos():
    rospy.init_node('is')
    pub_pose = rospy.Publisher("~vo", PoseWithCovarianceStamped, queue_size=100)
    rate = rospy.Rate(rate_publish) # 10hz
    while not rospy.is_shutdown():
        try: 
            position_reconstruction = getPoseReconstruction()
            quaternion = quaternion_from_euler(0, 0, position_reconstruction[2])
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.pose.position.x = position_reconstruction[0]
            pose.pose.pose.position.y = position_reconstruction[1]
            pose.pose.pose.orientation.z = quaternion[2]
            pose.pose.pose.orientation.w = quaternion[3]
            pose.pose.covariance = np.array([measurement_covariance_noise,   0,   0,   0,   0,   0,
                                            0,   measurement_covariance_noise,   0,   0,   0,   0,
                                            0,   0,   measurement_covariance_noise,   0,   0,   0,
                                            0,   0,   0,   measurement_covariance_noise,   0,   0,
                                            0,   0,   0,   0,   measurement_covariance_noise,   0,
                                            0,   0,   0,   0,   0,   measurement_covariance_noise])
            rospy.loginfo(pose)
            pub_pose.publish(pose)
            rate.sleep()
        except TypeError:
            pass

if __name__ == '__main__':

    broker_uri= rospy.get_param("is/broker_uri", "amqp://10.10.3.188:30000")
    aruco_id  =  rospy.get_param("is/aruco_id", 5)
    measurement_covariance_noise = rospy.get_param("is/measurement_covariance_noise", 0.14)
    rate_publish = rospy.get_param("is/rate_publish", 10)

    channel_recontruction = StreamChannel(broker_uri)
    subscription = Subscription(channel_recontruction)
    subscription.subscribe(topic=f"reconstruction.{aruco_id}.ArUco")

    try:
        publishToRos()
    except rospy.ROSInterruptException:
        pass