# About

This microsservice uses the detections of the [ArUco Detection](https://github.com/labviros/is-aruco-detector) to estimate the 3D poses of the Markers. To estimate the marker's pose, this microservice uses detections from all cameras that are viewing it.

# Dependencies:

[is-broker-events:](https://github.com/labviros/is-broker-events) Used to check which camera is available.

[is-aruco-detector:](https://github.com/labviros/is-aruco-detector) Used to get detections from each camera.

# Configuration

### Calibrations:

To change the calibration correctly in the deployment file, the following steps must be followed:

- Create a repository similar to this [link](https://github.com/vinihernech/camera-calibrations), where the calibration files for each camera must follow the name pattern `camera{camera_id}.json`

- In repository, put the link of the repository that contains the calibration

 ``` 
gitRepo:
  repository: "https://repository-with-calibration"
 ```
 - Change the directory to the location where the calibration is.
 
 ``` 
command: ["python"]
args: ["service.py", "../etc/config/options.json", "../etc/calibration/hd/path-where-is-the-calibration"]
 ```

# Usage

### Streams:

| Name | ⇒ Input | Output  ⇒ | Description |
| ---- | ------- | --------- | ----------- |
| reconstruction.ArUco | :incoming_envelope: **topic:** `ArUco.{camera_ids}.Detection` <br> :gem: **schema:** [ObjectAnnotations](https://github.com/labviros/is-msgs/tree/master/docs#is.vision.ObjectAnnotations) | :incoming_envelope: **topic:**  `reconstruction.{ArUco_id}.ArUco` <br> :gem: **schema:** [Pose](https://github.com/labviros/is-msgs/tree/master/docs#is.common.Pose) | Uses ArUco detector detections to estimate ArUco marker pose. |

# Examples:

In [exemples/consume/consume.py](https://github.com/matheusdutra0207/is-reconstruction/blob/main/exemples/consume/consume.py) is a simple consume to display the AruCo pose.

