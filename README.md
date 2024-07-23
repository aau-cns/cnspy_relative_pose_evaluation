# cnspy_relative_pose_evaluation

A python3 package for evaluating relative pose measurements between two spatial frames in order to assess the accuracy.
The baseline (ground truth) relative pose can be computed from two recorded 3D trajectories of the moving bodies and known extrinsics to the sensors.
These can be specified in a single configuration file, see [config.yaml](./test/sample_data/config.yaml)


The following evaluations can be conducted:

| Describtion    | Images |
|:---------:|:---:|
| Pose Error Plot (left: measured, right: error) | ![](./doc/img/Pose_Errors_ID0_to_1.png) |
| Range (gt vs. measured) outliers set to zero      | ![](./doc/img/Ranges_ID0.png) |
| Angle (gt vs. measured) outliers set to zero      | ![](./doc/img/Angle_ID0.png) |
| Range Error Histogram (filtered) and distribution | ![](./doc/img/Range_Error_Histograms_ID0.png) |
| Angle Error Histogram (filtered) and distribution | ![](./doc/img/Angle_Error_Histograms_ID0.png) |
| Statistics | [statistics.yaml](./doc/statistics.yaml) |


## Installation

Python 3.6 or greater is required. Inside this repo's directory, you may run
```
pip3 install .
```
or
``
pip3 install -e .
``
which installs the package in-place, allowing you make changes to the code without having to reinstall every time.

**This package is still in development. Once stable it should be sufficient to run:**
```commandline
pip3 install cnspy_relative_pose_evaluation
```
## Run from terminal

* RelPoseMeasEvaluationTool 
* RelPoseMeasEvaluation
* RelPose_ROSBag2CSV
* ROSBag_TrueRelPoses
* ROSBag_Poses2RelPose

## YAML configuration file

YAML configuration file is in the form of:
```yaml
# relative pose of the moving sensors
sensor_positions: {0:[0, 0, 0], 1:[0, 0, 0], 2:[0, 0, 0]}
sensor_orientations: {0:[1.0, 0, 0, 0], 1:[1.0, 0, 0, 0], 2:[1.0, 0, 0, 0]}
# true pose of the body
sensor_topics: {0: "/uav10/vrpn_client/raw_pose", 1: "/uav11/vrpn_client/raw_pose", 2: "/uav12/vrpn_client/raw_pose"}
# topcis of the relative pose measurement
relpose_topics: {0: "/uav10/data_handler/uvdar_fcu", 1: "/uav11/data_handler/uvdar_fcu", 2: "/uav12/data_handler/uvdar_fcu"}

```

## License

Software License Agreement (GNU GPLv3  License), refer to the LICENSE file.

*Sharing is caring!* - [Roland Jung](https://github.com/jungr-ait)  