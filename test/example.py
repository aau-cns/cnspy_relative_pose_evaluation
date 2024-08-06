#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2020, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
########################################################################################################################

import os

from cnspy_relative_pose_evaluation.ROSBag_Pose2AbsPoses import ROSBag_Pose2AbsPoses
from cnspy_relative_pose_evaluation.ROSBag_Poses2RelPoses import ROSBag_Poses2RelPoses
from cnspy_relative_pose_evaluation.ROSBag_TrueRelPoses import ROSBag_TrueRelPoses
from cnspy_relative_pose_evaluation.RelPose_ROSBag2CSV import RelPose_ROSBag2CSV
from cnspy_relative_pose_evaluation.RelPoseMeasEvaluationTool  import *
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_trajectory_evaluation.TrajectoryAlignmentTypes import TrajectoryAlignmentTypes

from cnspy_trajectory_evaluation.TrajectoryEvaluationTool import TrajectoryEvaluationTool

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')
RES_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_results')


class EvaluationTrail:

    @staticmethod
    def test_ROSBag_Poses2RelPose():
        sequence = 'static_test'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        ROSBag_Poses2RelPoses.extract(bagfile_name=bagfile_in,
                                      cfg=cfg_file,
                                      verbose=True,
                                      filename=None,
                                      result_dir=RES_DATA_DIR)

    @staticmethod
    def test_ROSBag_TrueRelPoses():
        sequence = 'static_test'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        bagfile_out = str(RES_DATA_DIR + '/' + sequence + '_gt.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        ROSBag_TrueRelPoses.extract(bagfile_in_name=bagfile_in,
                                    bagfile_out_name=bagfile_out,
                                     cfg=cfg_file,
                                     verbose=True)
    @staticmethod
    def test_ROSBag_TrueRelPoses_noise():
        sequence = 'static_test'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        bagfile_out = str(RES_DATA_DIR + '/' + sequence + '_noisy.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        ROSBag_TrueRelPoses.extract(bagfile_in_name=bagfile_in,
                                    bagfile_out_name=bagfile_out,
                                    cfg=cfg_file,
                                    stddev_pos=0.2,
                                    stddev_or=0.2,
                                    verbose=True)

    @staticmethod
    def test_RelPose_ROSbag2CSV():
        sequence = 'static_test'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        RelPose_ROSBag2CSV.extract(bagfile_name=bagfile_in,
                                     cfg=cfg_file,
                                     verbose=True)

        fn_out = str(RES_DATA_DIR + '/all_meas_relposes.csv')
        RelPose_ROSBag2CSV.extract_to_one(bagfile_name=bagfile_in,
                                   cfg=cfg_file,
                                   fn=fn_out,
                                   result_dir=RES_DATA_DIR,
                                   verbose=True)

    @staticmethod
    def test_RelPoseEval_same():

        sequence = 'static_test'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')

        fn_out = str(RES_DATA_DIR + '/all_meas_relposes.csv')
        RelPose_ROSBag2CSV.extract_to_one(bagfile_name=bagfile_in,
                                   cfg=cfg_file,
                                   fn=fn_out,
                                   result_dir=RES_DATA_DIR,
                                   with_cov=True,
                                   verbose=True)

        Sensor_ID_arr = [0,1,2]
        fn_gt_ranges = fn_out
        fn_meas_ranges = fn_out
        # 4) evaluate the ranges
        cfg = AssociateRelPoseCfg(ID1=None,
                                 ID2=None,
                                 relative_timestamps=False,
                                 max_difference=10**-4,
                                 subsample=0,
                                 verbose=True,
                                 remove_outliers=True,
                                 max_range=9,
                                 range_error_val=0,
                                 label_timestamp='t',
                                 label_ID1='ID1',
                                 label_ID2='ID2',
                                 label_range='range')
        cfg.max_range_error = 9

        eval = RelPoseMeasEvaluation(fn_gt=fn_gt_ranges,
                                     fn_est=fn_meas_ranges,
                                     ID1_arr=Sensor_ID_arr,
                                     ID2_arr=Sensor_ID_arr,
                                     cfg=cfg,
                                     result_dir=str(RES_DATA_DIR + "/" + sequence + '/eval_same/'),
                                     prefix='',
                                     save_plot=True,
                                     show_plot=False,
                                     save_statistics=True,
                                     plot_timestamps=True,
                                     plot_ranges=True,
                                     plot_angles=True,
                                     plot_ranges_sorted=True,
                                     plot_range_error=True,
                                     plot_angle_error=True,
                                     plot_range_histogram=True,
                                     plot_angle_histogram=True,
                                     filter_histogram=True,
                                     plot_pose_err=True,
                                     plot_pose=True,
                                     verbose=True
                                     )

    @staticmethod
    def test_RelPoseEval():
        sequence = 'two_spiral_2to3m'
        sequence = 'static_test'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')

        sequence = "two_spiral_one_hovers"
        bagfile_in = "/home/jungr/workspace/datasets/uvdar_dataset/merged_medium/two_spiral_one_hovers.bag"

        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        RelPoseMeasEvaluationTool.evaluate(bagfile_in=bagfile_in,
                                           cfg=cfg_file,
                                           result_dir=RES_DATA_DIR + "/" + sequence,
                                           save_plot=True,
                                           show_plot=False,
                                           verbose=True,
                                           ID_arr=[0,1,2])

    @staticmethod
    def test_RelPoseEval_gazebo():
        sequence = "dataset_octagon_loiter2_looped_merged_V2"
        bagfile_in = str("/home/jungr/workspace/catkin_ws/test_uvdar/src/aau/scripts/simulation/dataset_octagon_loiter2_looped_merged_sorted_2024-07-24-16-41-47.bag")

        cfg_file = str(SAMPLE_DATA_DIR + '/config_gazebo.yaml')
        RelPoseMeasEvaluationTool.evaluate(bagfile_in=bagfile_in,
                                           cfg=cfg_file,
                                           result_dir=RES_DATA_DIR + "/" + sequence,
                                           save_plot=True,
                                           show_plot=False,
                                           verbose=True,
                                           ID_arr=[1,2])

    @staticmethod
    def test_RelPoseEval_synthetic():
        sequence = 'static_test'
        bagfile_in = str(RES_DATA_DIR + '/' + sequence + '_noisy.bag')

        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        RelPoseMeasEvaluationTool.evaluate(bagfile_in=bagfile_in,
                                           cfg=cfg_file,
                                           result_dir=RES_DATA_DIR + "/" + sequence + "_noisy",
                                           save_plot=True,
                                           show_plot=False,
                                           filter_histogram=False,
                                           verbose=True,
                                           ID_arr=[0,1])

    @staticmethod
    def test_RPLIDAR():
        sequence = 'two_spiral_one_hovers'
        #bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        #bagfile_in = '/home/jungr/workspace/datasets/uvdar_dataset/sequences/static_test/static_test.bag'
        bagfile_in = '/home/jungr/workspace/datasets/uvdar_dataset/sequences/two_spiral_one_hovers/two_spiral_one_hovers.bag'
        cfg_file = str(SAMPLE_DATA_DIR + '/config-lidar.yaml')
        TrajectoryEvaluationTool.evaluate(bagfile_in=bagfile_in,
                                           cfg=cfg_file,
                                           result_dir=RES_DATA_DIR + "/" + sequence + "/Lidar/",
                                           alignment_type=TrajectoryAlignmentTypes.posyaw,
                                           est_err_type = EstimationErrorType.type1,
                                           num_aligned_samples=1,
                                           plot = True,
                                           save_plot=True,
                                           show_plot=False,
                                           max_difference=0.01,
                                           relative_timestamps=True,
                                          fmt=CSVSpatialFormatType.Pose2DStamped,
                                          IDs=[0,1,2],
                                           verbose=True)

    @staticmethod
    def test_RelPoseEval_synthetic_abspos():
        sequence = 'two_spiral_2to3m'
        sequence = 'static_test'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        bagfile_out = str(RES_DATA_DIR + '/' + sequence + '_abspose.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config-abspose.yaml')
        ROSBag_Pose2AbsPoses.extract(bagfile_in=bagfile_in,
                                      bagfile_out=bagfile_out,
                                      cfg=cfg_file,
                                      stddev_pos=0.1,
                                      stddev_or=0.1,
                                      use_header_timestamp=False,
                                      verbose=True)


if __name__ == "__main__":
     #EvaluationTrail.test_ROSBag_Poses2RelPose()
     #EvaluationTrail.test_ROSBag_TrueRelPoses()
     #EvaluationTrail.test_RelPose_ROSbag2CSV()
     #EvaluationTrail.test_ROSBag_TrueRelPoses_noise()
     #EvaluationTrail.test_RelPoseEval_same()
     #EvaluationTrail.test_RPLIDAR()
     EvaluationTrail.test_RelPoseEval_synthetic_abspos()

