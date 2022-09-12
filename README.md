# Is Reconstruction

This microsservice uses the detections of the [ArUco Detection](https://github.com/labviros/is-aruco-detector) to estimate the 3D poses of the Markers. 

## Dependencies:

[is-broker-events:](https://github.com/labviros/is-broker-events) Used to check which camera is available.

[is-aruco-detector:](https://github.com/labviros/is-aruco-detector) Used to get detections from each camera.



## Streams:

| Name | ⇒ Input | Output  ⇒ | Description |
| ---- | ------- | --------- | ----------- |
| reconstruction.ArUco | :incoming_envelope: **topic:** `ArUco.{camera_ids}.Detection` <br> :gem: **schema:** [ObjectAnnotations] | :incoming_envelope: **topic:**  `reconstruction.{ArUco_id}.ArUco` <br> :gem: **schema:** [Pose] | Uses ArUco detector detections to estimate ArUco marker pose. |

