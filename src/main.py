import sys
import socket
from is_wire.core import Message, Subscription, Logger, Channel
from is_msgs.image_pb2 import ObjectAnnotations, ObjectAnnotation
import numpy as np
from numpy.core.numeric import identity
from is_msgs.common_pb2 import ConsumerList, Pose
import json
from math import pi, cos, sin, acos
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


def sortCorners (aruco_id,corners):

    aruco_id = np.argsort(np.array([aruco_id]))
    center = np.zeros((aruco_id.size, 2))
    sorted_corners = []
    sorted_corners.append(corners[aruco_id[0]][:])
    center[0,:] = np.mean(corners[aruco_id[0]][0],axis=0) 
    return sorted_corners, center


def getArucoPose(detections, recontrued_points, reconstruction_only_one_camera):   

    if detections > 1:
        x = recontrued_points[0][0][0]
        y = recontrued_points[0][1][0]
        z = recontrued_points[0][2][0]
        x_p2p1 = recontrued_points[2][0][0] - recontrued_points[1][0][0]
        y_p2p1 = recontrued_points[2][1][0] - recontrued_points[1][1][0]
        yaw_rad = np.arctan2(y_p2p1, x_p2p1)
        yaw_deg = (180*yaw_rad)/pi  
        print(f"x = {x:.3f}, y = {y:.3f}, theta = {yaw_deg:.3f}, detect by = {detections}, cam = {detected_markers}")
        aruco_pose = Pose()
        aruco_pose.position.x = x
        aruco_pose.position.y = y
        aruco_pose.position.z = z
        aruco_pose.orientation.roll = 0
        aruco_pose.orientation.pitch = 0
        aruco_pose.orientation.yaw = yaw_rad

    elif detections == 1 and reconstruction_only_one_camera:
        x = recontrued_points[0][0][0]
        y = recontrued_points[0][1][0]
        x_p2p1 = recontrued_points[2][0][0] - recontrued_points[1][0][0]
        y_p2p1 = recontrued_points[2][1][0] - recontrued_points[1][1][0]
        yaw_rad = np.arctan2(y_p2p1, x_p2p1)
        yaw_deg = (180*yaw_rad)/pi
        print(f"x = {x:.3f}, y = {y:.3f}, theta = {yaw_deg:.3f}, detect by = {detections}, cam = {detected_markers}")
        aruco_pose = Pose()
        aruco_pose.position.x = x
        aruco_pose.position.y = y
        aruco_pose.position.z = aruco_height
        aruco_pose.orientation.roll = 0
        aruco_pose.orientation.pitch = 0
        aruco_pose.orientation.yaw = yaw_rad     

    return aruco_pose

def getReconstruedPoints(aruco_points_pixel, camera_params, detections):

    recontrued_points = []
    for aruco_point_pixel in aruco_points_pixel: 
        if detections > 1: # if more one camera saw the ArUco marker
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
                    m = np.array([aruco_point_pixel[0][index_x],aruco_point_pixel[0][index_y],1])
                    M[lin:(lin+3),col] = np.dot(camera_params[i].KRinv,m)
                    RinvT[lin:(lin+3),0] = camera_params[i].RinvT[0:3,0]
                    lin = lin + 3
                    col = col + 1                        
            M_inv = np.linalg.pinv(M)
            recontrued_point = np.dot(M_inv,RinvT)
            recontrued_points.append(recontrued_point)

        elif (detections == 1): # if only one camera saw the ArUco marker
            M = np.zeros((3,3))
            RinvT = np.zeros((3,1))
            aruco_height = 0.0
            for i in range(0, len(detected_markers[0])):
                if detected_markers[0][i] == 1:
                    id_camera = i + 1
                    index_x = id_camera + (id_camera - 2)
                    index_y = id_camera + (id_camera - 1)                             
                    M[0:2,0:2] = -identity(2)
                    m = np.array([aruco_point_pixel[0][index_x],aruco_point_pixel[0][index_y],1])
                    M[0:3,2] = np.dot(camera_params[i].KRinv,m)
                    RinvT[0:3,0] = camera_params[i].RinvT[0:3,0] + np.array([0,0,aruco_height])                                                     
            M_inv = np.linalg.inv(M)
            recontrued_point = np.dot(M_inv,RinvT)    
            recontrued_points.append(recontrued_point)        
    return recontrued_points

def arucoPointsPixel(calibrated_cameras_ids, subscriptions_aruco_detection):

    detected_markers = [np.zeros(calibrated_cameras_ids[-1], dtype=int)]
    centerAruco = [np.zeros(calibrated_cameras_ids[-1]*2, dtype=int)]
    cornerAruco_1 = [np.zeros(calibrated_cameras_ids[-1]*2, dtype=int)]
    cornerAruco_2 = [np.zeros(calibrated_cameras_ids[-1]*2, dtype=int)]    

    for subscription_aruco_detection in subscriptions_aruco_detection:
        reply = None
        try:
            reply = subscription_aruco_detection.channel.consume(timeout = time_wait_detection)
        except socket.timeout:
            pass
        
        if reply is not None:
            objectAnnotations = reply.unpack(ObjectAnnotations)
            if objectAnnotations.objects:
                for objectAnnotation in objectAnnotations.objects:
                    if objectAnnotation.id == detection_id:
                        vertices = objectAnnotation.region.vertices
                        vertices = np.array([[[[ vertices[0].x ,  vertices[0].y], 
                                                [vertices[1].x,  vertices[1].y], 
                                                [vertices[2].x,  vertices[2].y], 
                                                [vertices[3].x,  vertices[3].y] 
                                                ]]])
                        corners, center = sortCorners(detection_id, vertices)
                        detected_markers[0][subscription_aruco_detection.camera_id - 1] = 1  
                        index_x = subscription_aruco_detection.camera_id + (subscription_aruco_detection.camera_id - 2)
                        index_y = subscription_aruco_detection.camera_id + (subscription_aruco_detection.camera_id - 1) 
                        centerAruco[0][index_x] = center[0][0]
                        centerAruco[0][index_y] = center[0][1]

                        cornerAruco_1[0][index_x] = corners[0][0][0][0]
                        cornerAruco_1[0][index_y] = corners[0][0][0][1]

                        cornerAruco_2[0][index_x] = corners[0][0][1][0]
                        cornerAruco_2[0][index_y] = corners[0][0][1][1]                            

    aruco_points_pixel = [centerAruco, cornerAruco_1, cornerAruco_2]   
    detections = len(np.where(detected_markers[0][:] == 1)[0]) 
    return aruco_points_pixel, detections, detected_markers
   

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

        aruco_points_pixel, detections, detected_markers = arucoPointsPixel(calibrated_cameras_ids, subscriptions_aruco_detection)

        recontrued_points = getReconstruedPoints(aruco_points_pixel, camera_params, detections) 

        if detections > 1:
            aruco_pose = getArucoPose(detections, recontrued_points, reconstruction_only_one_camera)
            message = Message(content=aruco_pose)
            channel_to_publish.publish(message, topic=f"reconstruction.{detection_id}.ArUco")
