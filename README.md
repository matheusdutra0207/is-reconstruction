# Is Reconstruction

This microsservice uses the detections of the [ArUco Detection](https://github.com/labviros/is-aruco-detector) to estimate the 3D poses of the Markers. 


## Streams:

A stream is a program that consumes messages with a specific topic, processes them, and publishes messages in other topics, so if another service wants to use the informations provided by this service, it can simply subscribe to receive messages with the topic of interest.

| Name | ⇒ Input | Output  ⇒ | Description |
| ---- | ------- | --------- | ----------- |
| reconstruction.ArUco | :incoming_envelope: **topic:** `ArUco.{camera_ids}.Detection` <br> :gem: **schema:** [ObjectAnnotations] | :incoming_envelope: **topic:**  `reconstruction.{ArUco_id}.ArUco` <br> :gem: **schema:** [Pose] | Uses ArUco detector detections to estimate ArUco marker pose. |

