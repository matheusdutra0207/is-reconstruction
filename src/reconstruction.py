from is_wire.core import Logger
import numpy as np
from numpy.core.numeric import identity
from is_msgs.common_pb2 import Pose
from math import pi, cos, sin, acos


log = Logger(name="Reconstruction")


def getReconstruedPoints(aruco_points_pixel, 
                            camera_params, 
                            detections,
                            detected_markers,
                            aruco_height):

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

def getArucoPose(detections, 
                recontrued_points, 
                detected_markers,
                aruco_height):   
    if detections > 1:
        
        x = recontrued_points[0][0][0]
        y = recontrued_points[0][1][0]
        z = recontrued_points[0][2][0]
        x_p2p1 = recontrued_points[2][0][0] - recontrued_points[1][0][0]
        y_p2p1 = recontrued_points[2][1][0] - recontrued_points[1][1][0]
        yaw_rad = np.arctan2(y_p2p1, x_p2p1)
        yaw_deg = (180*yaw_rad)/pi  
        log.info(f"New pose estimated: x = {x:.3f}, y = {y:.3f}, theta = {yaw_deg:.3f}, recontrued by = {detections} cameras")
        aruco_pose = Pose()
        aruco_pose.position.x = x
        aruco_pose.position.y = y
        aruco_pose.position.z = z
        aruco_pose.orientation.roll = 0
        aruco_pose.orientation.pitch = 0
        aruco_pose.orientation.yaw = yaw_rad

    elif detections == 1:
        x = recontrued_points[0][0][0]
        y = recontrued_points[0][1][0]
        x_p2p1 = recontrued_points[2][0][0] - recontrued_points[1][0][0]
        y_p2p1 = recontrued_points[2][1][0] - recontrued_points[1][1][0]
        yaw_rad = np.arctan2(y_p2p1, x_p2p1)
        yaw_deg = (180*yaw_rad)/pi
        log.info(f"New pose estimated: x = {x:.3f}, y = {y:.3f}, theta = {yaw_deg:.3f}, recontrued by = {detections} cameras")
        aruco_pose = Pose()
        aruco_pose.position.x = x
        aruco_pose.position.y = y
        aruco_pose.position.z = aruco_height
        aruco_pose.orientation.roll = 0
        aruco_pose.orientation.pitch = 0
        aruco_pose.orientation.yaw = yaw_rad     

    return aruco_pose    