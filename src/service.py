import sys
import json
import glob

from is_wire.core import Message, Subscription, Logger, Channel
from is_msgs.image_pb2 import ObjectAnnotations, ObjectAnnotation

from subscription_manager import GetAvalibleCamera, SubscriptionDetection
from get_points_pixel import arucoPointsPixel
from reconstruction import getReconstruedPoints, getArucoPose
from camera_parameters import LoadCameraParameters


log = Logger(name="Service")

if __name__ == "__main__":
    
    config_file = sys.argv[1] if len(sys.argv) > 1 else '../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))
    broker_uri = config['broker_uri']
    channel_to_publish = Channel(broker_uri)
    detection_id = config['detection']['id']
    detection_type = config['detection']['detection_type']
    time_wait_detection = config['detection']['time_wait_for_detection']
    aruco_height = config['detection']['aruco_height']
    getAvalibleCamera = GetAvalibleCamera(broker_uri = broker_uri)
    camera_ids = getAvalibleCamera.run()
    calibrated_cameras_ids = []
    subscriptions_aruco_detection = []
    path_calibrations_list = glob.glob("../etc/calibration/hd/*")
    camera_params = []
    
    for cameras_id in camera_ids:
        for path_calibration in path_calibrations_list:
            if path_calibration.find(f'{cameras_id}') != -1:
                calibrated_cameras_ids.append(cameras_id)
    
    for camera_calibration_id in calibrated_cameras_ids:
        subscriptions_aruco_detection.append(
                        SubscriptionDetection(
                            broker_uri = broker_uri, 
                            detection_type = detection_type,
                            camera_id = camera_calibration_id))

    for calibrated_camera_ids in calibrated_cameras_ids:
        camera_params.append(LoadCameraParameters(calibration = f'../etc/calibration/hd/camera{calibrated_camera_ids}.json',
                            cameraID = calibrated_camera_ids))
   
    log.info(f"Available cameras for reconstruction: {calibrated_cameras_ids}")   

    while True:

        aruco_points_pixel, detections, detected_markers = arucoPointsPixel(calibrated_cameras_ids, 
                                                                        subscriptions_aruco_detection, 
                                                                        time_wait_detection,
                                                                        detection_id)

        recontrued_points = getReconstruedPoints(aruco_points_pixel, 
                                                    camera_params, 
                                                    detections,
                                                    detected_markers,
                                                    aruco_height) 

        if detections > 0:
            aruco_pose = getArucoPose(detections, 
                                    recontrued_points, 
                                    detected_markers,
                                    aruco_height)

            message = Message(content=aruco_pose)
            topic_to_publish = f"reconstruction.{detection_id}.ArUco"
            channel_to_publish.publish(message, topic= topic_to_publish)
            log.info(f"New pose published on: {topic_to_publish}") 
                                                        