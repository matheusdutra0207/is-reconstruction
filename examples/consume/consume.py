import numpy as np
import json
import socket

# IS
from is_wire.core import Channel, Subscription
from is_msgs.common_pb2 import Pose

if __name__ == '__main__':

    config_file = '../../etc/config/config.json'
    config = json.load(open(config_file, 'r'))
    broker_uri = config['broker_uri']
    detection_id = config['detection']['id']
    
    channel = Channel(broker_uri)
    subscription = Subscription(channel)
    subscription.subscribe(topic=f"reconstruction.{detection_id}.ArUco")
    while True:
        try:
            message = channel.consume(timeout = 0.01)
            pose = message.unpack(Pose)
            x_recontruction = pose.position.x
            y_recontruction = pose.position.y
            yaw_rad_recontruction = pose.orientation.yaw    
            print(f'x = {x_recontruction} m  y = {y_recontruction} m yaw = {(yaw_rad_recontruction*180)/np.pi} deg')

        except socket.timeout:
            pass