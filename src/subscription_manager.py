import re

from is_msgs.common_pb2 import ConsumerList
from is_wire.core import Message, Subscription, Logger, Channel

IS_BROKER_EVENTS_TOPIC = "BrokerEvents.Consumers"
RE_CAMERA_GATEWAY_CONSUMER = re.compile("CameraGateway\\.(\\d+)\\.GetConfig")

log = Logger(name="SubscriptionManager")


class GetAvalibleCamera:
    def __init__(self, broker_uri):
        self.channel = Channel(broker_uri)
        self.subscription = Subscription(self.channel)
        self.subscription.subscribe("BrokerEvents.Consumers")
        self.cameras = []

    def run(self):
        message = self.channel.consume()
        available_cameras = []
        consumers = message.unpack(ConsumerList)
        for key, _ in consumers.info.items():
            match = RE_CAMERA_GATEWAY_CONSUMER.match(key)
            if match is None:
                continue
            available_cameras.append(int(match.group(1)))

        available_cameras.sort()

        new_cameras = list(set(available_cameras) - set(self.cameras))

        self.cameras = list(available_cameras)
        return self.cameras

class SubscriptionDetection:
    def __init__(self, broker_uri, detection_type, camera_id):
        self.camera_id = camera_id
        self.channel = Channel(broker_uri)
        self.subscription = Subscription(self.channel)
        self.subscription.subscribe(f"{detection_type}.{camera_id}.Detection")