# About

This microsservice uses the detections of the [ArUco Detection](https://github.com/labviros/is-aruco-detector) to estimate the 3D poses of the Markers. To estimate the marker's pose, this microservice uses detections from all cameras that are viewing it.

# Dependencies

[is-broker-events:](https://github.com/labviros/is-broker-events) Used to check which camera is available.

[is-aruco-detector:](https://github.com/labviros/is-aruco-detector) Used to get detections from each camera.

# Configuration

### Calibrations

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

### Kubernetes

Make sure you have [kubectl](https://kubernetes.io/docs/tasks/tools/install-kubectl/) installed and the right `~/.kube/config` file to be able to interact with the cluster.

Deploy the stream application:

```bash
kubectl apply -f etc/k8s/deployment.yaml
```

The `.yaml` file describes two things:
* a deployment;
* a configmap;

A deployment is a way to run our application and guarantee that an N number of replicas will be running. The configmap allows load the options you desires when deploying into the kubernetes platform. See more about [deployment](https://kubernetes.io/docs/concepts/workloads/controllers/deployment/) and [confimap](https://kubernetes.io/docs/concepts/configuration/configmap/).

<!-- Links -->

[Image]: https://github.com/labviros/is-msgs/tree/master/docs/README.md#is.vision.Image
[ObjectAnnotations]: https://github.com/labviros/is-msgs/tree/master/docs/README.md#is.vision.ObjectAnnotations
[OpenCV]: https://docs.opencv.org/3.4.1/d7/d8b/tutorial_py_face_detection.html


### Streams

| Name | ⇒ Input | Output  ⇒ | Description |
| ---- | ------- | --------- | ----------- |
| reconstruction.ArUco | :incoming_envelope: **topic:** `ArUco.{camera_ids}.Detection` <br> :gem: **schema:** [ObjectAnnotations](https://github.com/labviros/is-msgs/tree/master/docs#is.vision.ObjectAnnotations) | :incoming_envelope: **topic:**  `reconstruction.{ArUco_id}.ArUco` <br> :gem: **schema:** [Pose](https://github.com/labviros/is-msgs/tree/master/docs#is.common.Pose) | Uses ArUco detector detections to estimate ArUco marker pose. |

# Examples

In [exemples/consume/consume.py](https://github.com/matheusdutra0207/is-reconstruction/blob/main/exemples/consume/consume.py) is a simple consume to display the AruCo pose.

