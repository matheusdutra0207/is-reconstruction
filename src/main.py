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

def camera_parameters(file):
    camera_data = json.load(open(file))
    K = np.array(camera_data['intrinsic']['doubles']).reshape(3, 3)
    res = [camera_data['resolution']['width'],
           camera_data['resolution']['height']]
    tf = np.array(camera_data['extrinsic']['tf']['doubles']).reshape(4, 4)
    #Rz = rotationZ_matrix(pi/2)[:3, :3]
    R = tf[:3, :3]
    #R = R@Rz
    T = tf[:3, 3].reshape(3, 1)
    dis = np.array(camera_data['distortion']['doubles'])
    return K, R, T, res, dis

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


    channel_publish = Channel(broker_uri)

    detection_id = config['detection']['id']
    detectionType = config['detection']['detection_type']
    reconstructionOnlyOneCamera = False if config['detection']['reconstruction_for_only_one_camera'] == "False" else True
    timeWaitDetection = config['detection']['time_wait_for_detection']
    getAvalibleCamera = GetAvalibleCamera(broker_uri = broker_uri)
    camera_ids = getAvalibleCamera.run()
    #camera_ids = [1, 2, 3, 4]
    camera_calibration_ids = []
    subscriptionsDetection = []
    path_calibration_list = glob.glob("../etc/calibration/hd/*")
    
    for cameras_id in camera_ids:
        for path_calibration in path_calibration_list:
            if path_calibration.find(f'{cameras_id}') != -1:
                camera_calibration_ids.append(cameras_id)
    
    for camera_calibration_id in camera_calibration_ids:
        subscriptionsDetection.append(
                        SubscriptionDetection(
                            broker_uri = broker_uri, 
                            detectionType = detectionType,
                            camera_id = camera_calibration_id))


    #Load cameras parameters
    cameraParams = []
    for cameraID in camera_calibration_ids:
        cameraParams.append(LoadCameraParameters(calibration = f'../etc/calibration/hd/camera{cameraID}.json',
                            cameraID = cameraID))

    print(camera_calibration_ids)                      

    K1, R1, T1, res1, dis1 = camera_parameters('../etc/calibration/hd/camera1.json')
    K2, R2, T2, res2, dis2 = camera_parameters('../etc/calibration/hd/camera2.json')
    K3, R3, T3, res3, dis3 = camera_parameters('../etc/calibration/hd/camera3.json')
    K4, R4, T4, res4, dis4 = camera_parameters('../etc/calibration/hd/camera4.json')


    while True:

        # detected_markers = np.array([[0, 0, 0, 0]])
        # centersAruco = np.array([[0, 0, 0, 0, 0, 0, 0, 0]])
        # cornerAruco_1 = np.array([[0, 0, 0, 0, 0, 0, 0, 0]])
        # cornerAruco_2 = np.array([[0, 0, 0, 0, 0, 0, 0, 0]])

        detected_markers = [np.zeros(camera_ids[-1], dtype=int)]
        centersAruco = [np.zeros(camera_ids[-1]*2, dtype=int)]
        cornerAruco_1 = [np.zeros(camera_ids[-1]*2, dtype=int)]
        cornerAruco_2 = [np.zeros(camera_ids[-1]*2, dtype=int)]    
        

        ################# consume localization (Pixel) #########################

        for subscriptionDetection in subscriptionsDetection:
            reply = None
            try:
                reply = subscriptionDetection.channel.consume(timeout = timeWaitDetection)
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
                            if subscriptionDetection.camera_id == 1:
                                centersAruco[0][0] = centers[0][0]
                                centersAruco[0][1] = centers[0][1]



                                cornerAruco_1[0][0] = corners[0][0][0][0]
                                cornerAruco_1[0][1] = corners[0][0][0][1]

                                cornerAruco_2[0][0] = corners[0][0][1][0]
                                cornerAruco_2[0][1] = corners[0][0][1][1]

                            elif subscriptionDetection.camera_id == 2:
                                centersAruco[0][2] = centers[0][0]
                                centersAruco[0][3] = centers[0][1]

                                cornerAruco_1[0][2] = corners[0][0][0][0]
                                cornerAruco_1[0][3] = corners[0][0][0][1]

                                cornerAruco_2[0][2] = corners[0][0][1][0]
                                cornerAruco_2[0][3] = corners[0][0][1][1]

                            elif subscriptionDetection.camera_id == 3:
                                centersAruco[0][4] = centers[0][0]
                                centersAruco[0][5] = centers[0][1]

                                cornerAruco_1[0][4] = corners[0][0][0][0]
                                cornerAruco_1[0][5] = corners[0][0][0][1]

                                cornerAruco_2[0][4] = corners[0][0][1][0]
                                cornerAruco_2[0][5] = corners[0][0][1][1]

                            elif subscriptionDetection.camera_id == 4:
                                centersAruco[0][6] = centers[0][0]
                                centersAruco[0][7] = centers[0][1]   

                                cornerAruco_1[0][6] = corners[0][0][0][0]
                                cornerAruco_1[0][7] = corners[0][0][0][1]

                                cornerAruco_2[0][6] = corners[0][0][1][0]
                                cornerAruco_2[0][7] = corners[0][0][1][1]

        arucoPoints_Pixel = [centersAruco, cornerAruco_1, cornerAruco_2] 

        ############################## Reconstruction ############################# 

            # Perform reconstruction
        KRinv_1 = np.linalg.inv(np.dot(K1,R1))
        RinvT_1 = np.dot(np.linalg.inv(R1),T1)
        KRinv_2 = np.linalg.inv(np.dot(K2,R2))
        RinvT_2 = np.dot(np.linalg.inv(R2),T2)
        KRinv_3 = np.linalg.inv(np.dot(K3,R3))
        RinvT_3 = np.dot(np.linalg.inv(R3),T3)
        KRinv_4 = np.linalg.inv(np.dot(K4,R4))
        RinvT_4 = np.dot(np.linalg.inv(R4),T4)    

        arucoPoints_World = []
        i = 1 ### Gambiarra 
        detections = len(np.where(detected_markers[i-1][:] == 1)[0])   

        for centers in arucoPoints_Pixel: ## Gambiarra
            if detections > 1:
                M = np.zeros((3*detections,detections+3))
                RinvT = np.zeros((3*detections,1))
                lin = 0 
                col = 3
                # if camera 1 detected the aruco marker i
                if detected_markers[i-1][0] == 1:
                    M[lin:(lin+3),0:3] = -identity(3)
                    m = np.array([centers[i-1][0],centers[i-1][1],1])
                    M[lin:(lin+3),col] = np.dot(KRinv_1,m)
                    RinvT[lin:(lin+3),0] = RinvT_1[0:3,0]
                    lin = lin + 3
                    col = col + 1
                # if camera 2 detected the aruco marker i
                if detected_markers[i-1][1] == 1:
                    M[lin:(lin+3),0:3] = -identity(3)
                    m = np.array([centers[i-1][2],centers[i-1][3],1])
                    M[lin:(lin+3),col] = np.dot(KRinv_2,m)
                    RinvT[lin:(lin+3),0] = RinvT_2[0:3,0]
                    lin = lin + 3
                    col = col + 1
                # if camera 3 detected the aruco marker i
                if detected_markers[i-1][2] == 1:
                    M[lin:(lin+3),0:3] = -identity(3)
                    m = np.array([centers[i-1][4],centers[i-1][5],1])
                    M[lin:(lin+3),col] = np.dot(KRinv_3,m)
                    RinvT[lin:(lin+3),0] = RinvT_3[0:3,0]
                    lin = lin + 3
                    col = col + 1
                # if camera 4 detected the aruco marker i    
                if detected_markers[i-1][3] == 1:
                    M[lin:(lin+3),0:3] = -identity(3)
                    m = np.array([centers[i-1][6],centers[i-1][7],1])
                    M[lin:(lin+3),col] = np.dot(KRinv_4,m)
                    RinvT[lin:(lin+3),0] = RinvT_4[0:3,0]
                    lin = lin + 3
                    col = col + 1

                # pseudo inverse of M
                M_inv = np.linalg.pinv(M)
                # Multiplication by RinvTs
                xyz = np.dot(M_inv,RinvT)
                arucoPoints_World.append(xyz)

            elif (detections == 1): 
                #print('just one detection for marker ',i)
                M = np.zeros((3,3))
                RinvT = np.zeros((3,1))
                d = 0.0
                # if camera 1 detected the aruco marker i
                if detected_markers[i-1][0] == 1:
                    M[0:2,0:2] = -identity(2)
                    m = np.array([centers[i-1][0],centers[i-1][1],1])
                    M[0:3,2] = np.dot(KRinv_1,m)
                    RinvT[0:3,0] = RinvT_1[0:3,0] + np.array([0,0,d])
                # if camera 2 detected the aruco marker i
                if detected_markers[i-1][1] == 1:
                    M[0:2,0:2] = -identity(2)
                    m = np.array([centers[i-1][2],centers[i-1][3],1])
                    M[0:3,2] = np.dot(KRinv_2,m)
                    RinvT[0:3,0] = RinvT_2[0:3,0] + np.array([0,0,d])
                # if camera 3 detected the aruco marker i
                if detected_markers[i-1][2] == 1:
                    M[0:2,0:2] = -identity(2)
                    m = np.array([centers[i-1][4],centers[i-1][5],1])
                    M[0:3,2] = np.dot(KRinv_3,m)
                    RinvT[0:3,0] = RinvT_3[0:3,0] + np.array([0,0,d])
                # if camera 4 detected the aruco marker i
                if detected_markers[i-1][3] == 1:
                    M[0:2,0:2] = -identity(2)
                    m = np.array([centers[i-1][6],centers[i-1][7],1])
                    M[0:3,2] = np.dot(KRinv_4,m)
                    RinvT[0:3,0] = RinvT_4[0:3,0] + np.array([0,0,d])
                # inverse of M
                M_inv = np.linalg.inv(M)
                # Multiplication by RinvT
                xyz = np.dot(M_inv,RinvT)    
                arucoPoints_World.append(xyz) 

        ######################### pub ########################

        ######################### gambiarra #################
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
            channel_publish.publish(message, topic=f"localization.{detection_id}.aruco")


        elif detections == 1 and reconstructionOnlyOneCamera:
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
            channel_publish.publish(message, topic=f"localization.{detection_id}.aruco")
