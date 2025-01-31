# cnspy_relative_pose_evaluation

A python3 package for evaluating uncertain relative pose measurements between the spatial frame of sensors to other sensors or static objects in order to assess the accuracy.
The baseline (ground truth) relative pose can be computed from two recorded 3D trajectories of the moving bodies and known extrinsics to the sensors.
These can be specified in a single configuration file, see [config.yaml](./test/sample_data/config.yaml)

The relative measurements can be either provided as CSV files or in ROS1 bagfile in [PoseWithCovarianceArrayStamped](https://ctu-mrs.github.io/mrs_msgs/msg/PoseWithCovarianceArrayStamped.html) msgs.
The true relative pose measurements can be either provided as CSV files or can be computed from measured true pose of the BODY reference frame and a provided pose in a ROS1 bagfile (`geometry_msgs/PoseStamped`, `geometry_msgs/TransformStamped`) between the BODY reference and the SENSOR reference frame (specified in the configuration file) . 


The following evaluations can be conducted:

| Describtion    | Images |
|:---------:|:---:|
| Pose Plot (left: measured, right: gt) | ![](./doc/img/Pose_ID1_to_2.png) |
| Pose Error Plot (left: position, right: orientation, top: meas, middle: error, bottom: nees)  | ![](./doc/img/Pose_Errors_ID1_to_2.png) |
| Range (gt vs. measured) outliers removed      | ![](./doc/img/Ranges_ID1.png) |
| Range Sorted (gt vs. measured) outliers removed      | ![](./doc/img/Range_Sorted_ID1.png) |
| Range Error (measured-gt) outliers removed      | ![](./doc/img/Range_Errors_ID1.png) |
| Range Error Histogram (filtered) and distribution | ![](./doc/img/Range_Error_Histograms_ID1.png) |
| Angle (gt vs. measured) outliers set to zero      | ![](./doc/img/Angle_ID1.png) |
| Angle Error (measured-gt) outliers removed      | ![](./doc/img/Angle_Errors_ID1.png) |
| Angle Error Histogram (filtered) and distribution | ![](./doc/img/Angle_Error_Histograms_ID1.png) |
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

**Note that the ROS1 message definitions from the [mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system?tab=readme-ov-file#native-installation) need to be installed:**
```commandline
curl https://ctu-mrs.github.io/ppa-stable/add_ppa.sh | bash
sudo apt install ros-noetic-mrs-msgs
```

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
# relative pose of the moving sensors with respect to the body frame (pose from BODY to SENSOR)
sensor_positions: {0:[0, 0, 0], 1:[0, 0, 0], 2:[0, 0, 0]}
sensor_orientations: {0:[1.0, 0, 0, 0], 1:[1.0, 0, 0, 0], 2:[1.0, 0, 0, 0]}
# static pose of stationary objects with respect to the GLOBAL frame
object_positions: {10:[1, 0, 0], 11:[2, 0, 0], 12:[3, 0, 0]}
object_orientations: {10:[1.0, 0, 0, 0], 11:[1.0, 0, 0, 0], 12:[1.0, 0, 0, 0]}
# true pose of the BODY (geometry_msgs/PoseStamped, geometry_msgs/TransformStamped) with respect to the GLOBAL frame
true_pose_topics: {0: "/uav10/vrpn_client/raw_pose", 1: "/uav11/vrpn_client/raw_pose", 2: "/uav12/vrpn_client/raw_pose"}
# topics of the relative pose measurement (mrs_msgs/PoseWithCovarianceArrayStamped)
relpose_topics: {0: "/uav10/data_handler/uvdar_fcu", 1: "/uav11/data_handler/uvdar_fcu", 2: "/uav12/data_handler/uvdar_fcu"}
# topics of the synthetic relative pose measurement (mrs_msgs/PoseWithCovarianceArrayStamped) to be stored in a "clean" rosbag file
new_relpose_topics: {0: "/uav10/data_handler/uvdar_fcu_syn", 1: "/uav11/data_handler/uvdar_fcu_syn", 2: "/uav12/data_handler/uvdar_fcu_syn"}
```
Note that the IDs need to be unique and match with the IDs in the messages of the `relpose_topics`. 
The `RelPoseMeasEvaluationTool` checks if the specified topics are contained in the provided bagfile and if the IDs in the cfg file are unique.

## Important notes for the NEES computation

The covariance of the measurements are assumed to represent the position uncertainty in meters and the orientation uncertainty in radians. Currently, three perturbation types are supported `EstimationErrorType.type1`, `EstimationErrorType.type2`, or `EstimationErrorType.type5`.

type1:
[<img src="/doc/img/e-type1.png" width="250"/>](./doc/img/e-type1.png)

type2:
[<img src="/doc/img/e-type2.png" width="250"/>](./doc/img/e-type2.png)

type5:
[<img src="/doc/img/e-type5.png" width="250"/>](./doc/img/e-type5.png)

For more details, please read: [ErrorRepresentationType](https://github.com/aau-cns/cnspy_spatial_csv_formats/blob/main/cnspy_spatial_csv_formats/ErrorRepresentationType.py) and [EstimationErrorType](https://github.com/aau-cns/cnspy_spatial_csv_formats/blob/main/cnspy_spatial_csv_formats/EstimationErrorType.py)

## Usage

```commandline
usage: RelPoseMeasEvaluationTool [-h] [--result_dir RESULT_DIR] [--bagfile BAGFILE] --cfg CFG [--save_plot] [--show_plot] [--verbose] [--extra_plots] [--keep_outliers] [--filter_histogram] [--max_range MAX_RANGE] [--max_angle MAX_ANGLE] [--interpolation_type {cubic,linear}] [--min_dt MIN_DT]
                                 [--pose_error_type {type1,type2,type3,type4,type5,type6,none}]

RelPoseMeasEvaluationTool: evaluation the measured relative poses

optional arguments:
  -h, --help            show this help message and exit
  --result_dir RESULT_DIR
                        directory to store results [otherwise bagfile name will be a directory]
  --bagfile BAGFILE     input bag file
  --cfg CFG             YAML configuration file describing the setup: {sensor_positions:{<id>:[x,y,z], ...}, sensor_orientations:{<id>:[w,x,y,z], ...}, relpose_topics:{<id>:<topic_name>, ...}, true_pose_topics:{<id>:<topic_name>, ...}
  --save_plot           saves all plots in the result_dir
  --show_plot           blocks the evaluation, for inspecting individual plots, continuous after closing
  --verbose
  --extra_plots         plots: timestamps, ranges + angles (sorted + unsorted + error),
  --keep_outliers       do not apply the max. thresholds on the error
  --filter_histogram    filters the error histogram, such that the fitted normal distribution is computed on the best bins only
  --max_range MAX_RANGE
                        max. range that classifies them as outlier (0 disables feature).
  --max_angle MAX_ANGLE
                        max. range that classifies them as outlier (0 disables feature)
  --interpolation_type {cubic,linear}
                        Trajectory interpolation type
  --min_dt MIN_DT       temporal displacement of cubic spline control points
  --pose_error_type {type1,type2,type3,type4,type5,type6,none}
                        Covariance perturbation type (space) of relative pose measurements
```

Other tools:

- `RelPoseMeasEvaluation`,
- `RelPose_ROSBag2CSV`,
- `ROSBag_TrueRelPoses`,
- `ROSBag_Poses2RelPoses`,
- `ROSBag_ModifyRelPoses`,
- `ROSBag_Pose2AbsPoses`

## License

Software License Agreement (GNU GPLv3  License), refer to the LICENSE file.

*Sharing is caring!* - [Roland Jung](https://github.com/jungr-ait)  