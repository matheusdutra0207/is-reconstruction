import numpy as np
import json


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