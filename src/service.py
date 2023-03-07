import sys
import json
import glob
import numpy as np

from is_wire.core import Message, Subscription, Logger, Channel
from is_msgs.image_pb2 import ObjectAnnotations, ObjectAnnotation

from reconstruction.subscription_manager import GetAvalibleCamera, SubscriptionDetection
from reconstruction.get_points_pixel import arucoPointsPixel
#from reconstruction import getReconstruedPoints, getArucoPose
from reconstruction.reconstruction import Reconstruction
from reconstruction.camera_parameters import LoadCameraParameters

import time 
log = Logger(name="Service")


def main():
    config_file = sys.argv[1] if len(sys.argv) > 1 else '../etc/config/config.json'
    config = json.load(open(config_file, 'r'))
    broker_uri = config['broker_uri']
    channel_to_publish = Channel(broker_uri)
    detection_id = config['detection']['id']
    detection_type = config['detection']['detection_type']
    time_wait_detection = config['detection']['time_wait_for_detection']
    aruco_height = config['detection']['aruco_height']
    estimate_position_with_only_one_detection = 0 if config['detection']['estimate_position_with_only_one_detection']== "True" else 1
    getAvalibleCamera = GetAvalibleCamera(broker_uri = broker_uri)
    camera_ids = getAvalibleCamera.run()
    calibrated_cameras_ids = []
    subscriptions_aruco_detection = []
    path_calibrations = sys.argv[2] if len(sys.argv) > 2 else "../etc/calibration/hd/"
    path_calibrations_list = glob.glob(f'{path_calibrations}*')
    camera_params = []
    # camera_params =  camera_params.append(LoadCameraParameters(calibration = f'{path_calibrations}camera{3}.json',
    #             cameraID = 3))


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

    camera_params = list(np.zeros(calibrated_cameras_ids[-1] + 1, dtype=int)) 
    for calibrated_camera_id in calibrated_cameras_ids:  
            camera_params[calibrated_camera_id] = LoadCameraParameters(calibration = f'{path_calibrations}camera{calibrated_camera_id}.json',
                                cameraID = calibrated_camera_id)

    log.info(f"Available cameras for reconstruction: {calibrated_cameras_ids}")   
    reconstruction = Reconstruction()

    while True:
        start_loop = time.time()
        
        aruco_points_pixel, detections, detected_markers = arucoPointsPixel(calibrated_cameras_ids, 
                                                                        subscriptions_aruco_detection, 
                                                                        time_wait_detection,
                                                                        detection_id)

        recontrued_points = reconstruction.getReconstruedPoints(aruco_points_pixel, 
                                                    camera_params, 
                                                    detections,
                                                    detected_markers,
                                                    aruco_height) 

        if detections > estimate_position_with_only_one_detection:
            aruco_pose = reconstruction.getArucoPose(detections, 
                                    recontrued_points, 
                                    detected_markers,
                                    aruco_height,
                                    start_loop)

            message = Message(content=aruco_pose)
            topic_to_publish = f"localization.{detection_id}.aruco"
            channel_to_publish.publish(message, topic= topic_to_publish)

            #log.info(f"New pose published on: {topic_to_publish}") 
                                                        
if __name__ == "__main__":
    main()