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
from is_msgs.camera_pb2 import FrameTransformation

#  my own
from streamChannel import StreamChannel


def getPoseReconstruction():
    message = channel_recontruction.consume_last()
    if type(message) != bool:
        f = message.unpack(FrameTransformation)
        tf = f.tf.doubles
        x_recontruction = tf[0]
        y_recontruction = tf[1]
        roll_rad_recontruction = tf[3]

        return np.array([x_recontruction, y_recontruction, roll_rad_recontruction])
    else:
        return message


def publishToRos():
    rospy.init_node('is')
    pub_pose = rospy.Publisher("~vo", PoseWithCovarianceStamped, queue_size=100)
    rate = rospy.Rate(10) # 10hz
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


            covarianceIs = 0.14

            pose.pose.covariance = np.array([covarianceIs,   0,   0,   0,   0,   0,
                                    0,   covarianceIs,   0,   0,   0,   0,
                                    0,   0,   covarianceIs,   0,   0,   0,
                                    0,   0,   0,   covarianceIs,   0,   0,
                                    0,   0,   0,   0,   covarianceIs,   0,
                                    0,   0,   0,   0,   0,   covarianceIs])
            rospy.loginfo(pose)
            pub_pose.publish(pose)
            rate.sleep()
        except TypeError:
            pass

if __name__ == '__main__':
    channel_recontruction = StreamChannel("amqp://10.10.3.188:30000")
    subscription = Subscription(channel_recontruction)

    aruco_id = 5
    subscription.subscribe(topic=f"localization.{aruco_id}.aruco")

    try:
        publishToRos()
    except rospy.ROSInterruptException:
        pass