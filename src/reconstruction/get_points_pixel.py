import socket
import numpy as np

from is_msgs.image_pb2 import ObjectAnnotations, ObjectAnnotation
from numpy.core.numeric import identity



def sortCorners(aruco_id,corners):

    aruco_id = np.argsort(np.array([aruco_id]))
    center = np.zeros((aruco_id.size, 2))
    sorted_corners = []
    sorted_corners.append(corners[aruco_id[0]][:])
    center[0,:] = np.mean(corners[aruco_id[0]][0],axis=0) 
    return sorted_corners, center


def arucoPointsPixel(calibrated_cameras_ids, 
                    subscriptions_aruco_detection, 
                    time_wait_detection,
                    detection_id):
  
    detected_markers = [np.zeros(calibrated_cameras_ids[-1] + 1, dtype=int)]
    centerAruco = [np.zeros(calibrated_cameras_ids[-1]*2 + 2, dtype=int)]
    cornerAruco_1 = [np.zeros(calibrated_cameras_ids[-1]*2 + 2 , dtype=int)]
    cornerAruco_2 = [np.zeros(calibrated_cameras_ids[-1]*2 + 2, dtype=int)]    

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
                        detected_markers[0][subscription_aruco_detection.camera_id] = 1
                        index_x = subscription_aruco_detection.camera_id*2   
                        index_y = subscription_aruco_detection.camera_id*2 + 1 

                        centerAruco[0][index_x] = center[0][0]
                        centerAruco[0][index_y] = center[0][1]

                        cornerAruco_1[0][index_x] = corners[0][0][0][0]
                        cornerAruco_1[0][index_y] = corners[0][0][0][1]

                        cornerAruco_2[0][index_x] = corners[0][0][1][0]
                        cornerAruco_2[0][index_y] = corners[0][0][1][1] 

                
    aruco_points_pixel = [centerAruco, cornerAruco_1, cornerAruco_2] 
    detections = len(np.where(detected_markers[0][:] == 1)[0])
    return aruco_points_pixel, detections, detected_markers