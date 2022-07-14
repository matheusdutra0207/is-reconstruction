#!/usr/bin/env python3
# license removed for brevity

# Python
import numpy as np
import random

# ROS
import rospy
from geometry_msgs.msg import PoseWithCovariance

# IS
from is_wire.core import Channel, Subscription
import numpy as np
from is_msgs.camera_pb2 import FrameTransformation

#  my own
from stream_channel import StreamChannel


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


def publishTORos():
    rospy.init_node('is')
    pub_pose = rospy.Publisher("~vo", PoseWithCovariance, queue_size=100)
    rate = rospy.Rate(15) # 20hz
    while not rospy.is_shutdown():
        try: 
            position_reconstruction = getPoseReconstruction()

            pose = PoseWithCovariance()
            pose.pose.position.x = position_reconstruction[0]
            pose.pose.position.y = position_reconstruction[1]
            pose.pose.orientation.z = position_reconstruction[2]
            covarianceIs = 0.14

            pose.covariance = np.array([covarianceIs,   0,   0,   0,   0,   0,
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
        publishTORos()
    except rospy.ROSInterruptException:
        pass