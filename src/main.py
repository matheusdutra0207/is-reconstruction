import sys
import socket
from is_msgs.common_pb2 import ConsumerList
from is_wire.core import Message, Subscription, Logger, Channel
from is_wire.rpc import ServiceProvider, LogInterceptor
from is_msgs.image_pb2 import ObjectAnnotations, ObjectAnnotation, Vertex

import numpy as np
from numpy.core.numeric import identity
from google.protobuf.json_format import Parse
from is_wire.core.utils import now
from is_msgs.robot_pb2 import RobotTaskRequest, RobotTaskReply
from is_msgs.common_pb2 import ConsumerList
import json
from math import pi, cos, sin, acos
from is_msgs.camera_pb2 import FrameTransformations, FrameTransformation

from is_msgs.common_pb2 import Pose
import math
from math import pi

from subscription_manager import GetAvalibleCamera, SubscriptionDetection
import glob

class LoadCameraParameters:

    def __init__(self, calibration, cameraID): 
        camera_data = json.load(open(calibration))
        self.cameraID = cameraID
        self.K = np.array(camera_data['intrinsic']['doubles']).reshape(3, 3)
        self.res = [camera_data['resolution']['width'],
                    camera_data['resolution']['height']]
        self.tf = np.array(camera_data['extrinsic']['tf']['doubles']).reshape(4, 4)
        self.R = self.tf[:3, :3]
        self.T = self.tf[:3, 3].reshape(3, 1)
        self.dis = np.array(camera_data['distortion']['doubles'])    
        self.KRinv = np.linalg.inv(np.dot(self.K, self.R))
        self.RinvT = np.dot(np.linalg.inv(self.R), self.T)            


def sort_corners (ids,corners):
    ind = np.argsort(ids[:,0])
    centers = np.zeros((ind.size,2))
    ids = ids[ind]
    sorted_corners = []
    
    for a in range(ind.size):
        sorted_corners.append(corners[ind[a]][:])
        centers[a,:] = np.mean(corners[ind[a]][0],axis=0) 
    return sorted_corners, centers

if __name__ == "__main__":
    
    config_file = sys.argv[1] if len(sys.argv) > 1 else '../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))
    broker_uri = config['broker_uri']
    channel_to_publish = Channel(broker_uri)
    detection_id = config['detection']['id']
    detection_type = config['detection']['detection_type']
    reconstruction_only_one_camera = False if config['detection']['reconstruction_for_only_one_camera'] == "False" else True
    time_wait_detection = config['detection']['time_wait_for_detection']
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

    print(calibrated_cameras_ids)                    

    while True:
        detected_markers = [np.zeros(calibrated_cameras_ids[-1], dtype=int)]
        centersAruco = [np.zeros(calibrated_cameras_ids[-1]*2, dtype=int)]
        cornerAruco_1 = [np.zeros(calibrated_cameras_ids[-1]*2, dtype=int)]
        cornerAruco_2 = [np.zeros(calibrated_cameras_ids[-1]*2, dtype=int)]    

        for subscriptionDetection in subscriptions_aruco_detection:
            reply = None
            try:
                reply = subscriptionDetection.channel.consume(timeout = time_wait_detection)
            except socket.timeout:
                pass
            
            if reply is not None:
                objectAnnotations = reply.unpack(ObjectAnnotations)
                if objectAnnotations.objects:
                    for objectAnnotation in objectAnnotations.objects:
                        if objectAnnotation.id == detection_id:
                            vertices = objectAnnotation.region.vertices
                            corners = np.array([[[[ vertices[0].x ,  vertices[0].y], 
                                                    [vertices[1].x,  vertices[1].y], 
                                                    [vertices[2].x,  vertices[2].y], 
                                                    [vertices[3].x,  vertices[3].y] 
                                                    ]]])
                            corners, centers = sort_corners(np.array([[detection_id]]), corners)
                            detected_markers[0][subscriptionDetection.camera_id - 1] = 1  
                            index_x = subscriptionDetection.camera_id + (subscriptionDetection.camera_id - 2)
                            index_y = subscriptionDetection.camera_id + (subscriptionDetection.camera_id - 1) 
                            centersAruco[0][index_x] = centers[0][0]
                            centersAruco[0][index_y] = centers[0][1]

                            cornerAruco_1[0][index_x] = corners[0][0][0][0]
                            cornerAruco_1[0][index_y] = corners[0][0][0][1]

                            cornerAruco_2[0][index_x] = corners[0][0][1][0]
                            cornerAruco_2[0][index_y] = corners[0][0][1][1]                            

        arucoPoints_Pixel = [centersAruco, cornerAruco_1, cornerAruco_2]   

        arucoPoints_World = []
        detections = len(np.where(detected_markers[0][:] == 1)[0])   

        for centers in arucoPoints_Pixel: 
            if detections > 1:
                M = np.zeros((3*detections,detections+3))
                RinvT = np.zeros((3*detections,1))
                lin = 0 
                col = 3
                for i in range(0, len(detected_markers[0])):
                    if detected_markers[0][i] == 1:
                        id_camera = i + 1
                        index_x = id_camera + (id_camera - 2)
                        index_y = id_camera + (id_camera - 1)                        
                        M[lin:(lin+3),0:3] = -identity(3)
                        m = np.array([centers[0][index_x],centers[0][index_y],1])
                        M[lin:(lin+3),col] = np.dot(camera_params[i].KRinv,m)
                        RinvT[lin:(lin+3),0] = camera_params[i].RinvT[0:3,0]
                        lin = lin + 3
                        col = col + 1                        
                M_inv = np.linalg.pinv(M)
                xyz = np.dot(M_inv,RinvT)
                arucoPoints_World.append(xyz)

            elif (detections == 1): 
                M = np.zeros((3,3))
                RinvT = np.zeros((3,1))
                aruco_height = 0.0
                for i in range(0, len(detected_markers[0])):
                    if detected_markers[0][i] == 1:
                        id_camera = i + 1
                        index_x = id_camera + (id_camera - 2)
                        index_y = id_camera + (id_camera - 1)                             
                        M[0:2,0:2] = -identity(2)
                        m = np.array([centers[0][index_x],centers[0][index_y],1])
                        M[0:3,2] = np.dot(camera_params[i].KRinv,m)
                        RinvT[0:3,0] = camera_params[i].RinvT[0:3,0] + np.array([0,0,aruco_height])                                                     
                M_inv = np.linalg.inv(M)
                xyz = np.dot(M_inv,RinvT)    
                arucoPoints_World.append(xyz) 

        if detections > 1:
            x = arucoPoints_World[0][0][0]
            y = arucoPoints_World[0][1][0]
            z = arucoPoints_World[0][2][0]
            x_p2p1 = arucoPoints_World[2][0][0] - arucoPoints_World[1][0][0]
            y_p2p1 = arucoPoints_World[2][1][0] - arucoPoints_World[1][1][0]
            roll_rad = np.arctan2(y_p2p1, x_p2p1)
            roll_deg = (180*roll_rad)/pi  
            print(f"x = {x:.3f}, y = {y:.3f}, theta = {roll_deg:.3f}, detect by = {detections}, cam = {detected_markers}")
            f = FrameTransformation()          
            f.tf.doubles.append(x)
            f.tf.doubles.append(y)
            f.tf.doubles.append(z)
            f.tf.doubles.append(roll_rad)
            f.tf.doubles.append(0.000025)
            f.tf.doubles.append(0.2)

            message = Message(content=f)
            channel_to_publish.publish(message, topic=f"localization.{detection_id}.aruco")


        elif detections == 1 and reconstruction_only_one_camera:
            x = arucoPoints_World[0][0][0]
            y = arucoPoints_World[0][1][0]
            x_p2p1 = arucoPoints_World[2][0][0] - arucoPoints_World[1][0][0]
            y_p2p1 = arucoPoints_World[2][1][0] - arucoPoints_World[1][1][0]
            roll_rad = np.arctan2(y_p2p1, x_p2p1)
            roll_deg = (180*roll_rad)/pi
            print(f"x = {x:.3f}, y = {y:.3f}, theta = {roll_deg:.3f}, detect by = {detections}, cam = {detected_markers}")


            f = FrameTransformation()
            f.tf.doubles.append(x)
            f.tf.doubles.append(y)
            f.tf.doubles.append(roll_rad)
            f.tf.doubles.append(roll_rad)
            f.tf.doubles.append(0.2)
            f.tf.doubles.append(0.2)

            message = Message(content=f)
            channel_to_publish.publish(message, topic=f"localization.{detection_id}.aruco")
