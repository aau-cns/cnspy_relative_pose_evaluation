#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2024, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
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
# BASED ON: https://github.com/aau-cns/cnspy_relative_pose_evaluation
# just install "pip install cnspy_relative_pose_evaluation"
########################################################################################################################

import rosbag
import time
import os
import argparse
import yaml
import csv
from tqdm import tqdm
import numpy as np
from numpy import linalg as LA
from spatialmath import UnitQuaternion, SO3, SE3, Quaternion, base

from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from std_msgs.msg import Header, Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped

from cnspy_trajectory.BsplineSE3 import TrajectoryInterpolationType
from cnspy_relative_pose_evaluation.ROSBag_Poses2RelPoses import *
from cnspy_relative_pose_evaluation.ROSBag_TrueRelPoses import *
from cnspy_relative_pose_evaluation.RelPose_ROSBag2CSV import *
from cnspy_relative_pose_evaluation.AssociateRelPoses import AssociateRelPoses, AssociateRelPoseCfg
from cnspy_relative_pose_evaluation.RelPoseMeasEvaluation import RelPoseMeasEvaluation


class RelPoseMeasEvaluationTool:
    @staticmethod
    def evaluate(bagfile_in:str,
                 cfg_fn:str,
                 result_dir=None,
                 verbose=True,
                 show_plot=True,
                 save_plot=True,
                 ID_arr=None,
                 filter_histogram=True,
                 extra_plots=True,
                 max_range=None,
                 max_angle=None,
                 remove_outliers=True,
                 interp_type: TrajectoryInterpolationType = TrajectoryInterpolationType.linear,
                 min_dt=0.05,
                 pose_error_type: EstimationErrorType = EstimationErrorType.type5):
        if not os.path.isfile(bagfile_in):
            print("RangeEvaluationTool: could not find file: %s" % bagfile_in)
            return False

        ## create result dir:
        if result_dir == "" or result_dir is None:
            folder = str(bagfile_in).replace(".bag", "")
        else:
            folder = result_dir

        folder = os.path.abspath(folder)
        try:  # else already exists
            os.makedirs(folder)
        except:
            pass
        result_dir = folder
        if verbose:
            print("* result_dir: \t " + str(folder))

        dict_cfg = None
        with open(cfg_fn, "r") as yamlfile:
            dict_cfg = yaml.load(yamlfile, Loader=yaml.FullLoader)
            if "true_pose_topics" not in dict_cfg:
                print("[true_pose_topics] does not exist in fn=" + cfg_fn)
                return False
            if "relpose_topics" not in dict_cfg:
                print("[relpose_topics] does not exist in fn=" + cfg_fn)
                return False
            # if "new_relpose_topics" not in dict_cfg:
            #    print("[new_relpose_topics] was specified and is currently NOT SUPPORTED (please remove or comment it out) in fn=" + cfg_fn)
            #    return False
            print("Successfully read YAML config file.")

        Sensor_ID_arr = []
        Object_ID_arr = []
        topic_list = []
        for key, val in dict_cfg["true_pose_topics"].items():
            topic_list.append(val)
        for key, val in dict_cfg["relpose_topics"].items():
            topic_list.append(val)
            Sensor_ID_arr.append(int(key))
        if "object_positions" in dict_cfg:
            for key, val in dict_cfg["object_positions"].items():
                Object_ID_arr.append(int(key))

        if ID_arr:
            Sensor_ID_arr = ID_arr

        # unique IDs
        Sensor_ID_arr = list(set(Sensor_ID_arr))
        Object_ID_arr = list(set(Object_ID_arr))
        if verbose:
            print("* topic_list= " + str(topic_list))
            print("* Sensor_ID_arr= " + str(Sensor_ID_arr))
            print("* Object_ID_arr= " + str(Sensor_ID_arr))

        # topic_list=['/d01/ranging', '/a01/ranging', '/a02/ranging', '/a03/ranging']
        fn_meas_ranges = str(result_dir + '/all-meas-ranges.csv')
        fn_gt_ranges = str(result_dir + '/all-true-ranges.csv')

        IDs_found = None
        # 1) extract all measurements to CSV
        if not os.path.isfile(fn_meas_ranges):
            res, IDs_found = RelPose_ROSBag2CSV.extract_to_one(bagfile_name=bagfile_in,
                                                    cfg=cfg_fn,
                                                    fn=fn_meas_ranges,
                                                    result_dir=result_dir,
                                                    verbose=verbose,
                                                    with_cov=True)

        bagfile_out = str(result_dir + '/true-ranges.bag')

        # 2) create a clean bag file
        if not os.path.isfile(bagfile_out):
            res= ROSBag_TrueRelPoses.extract(bagfile_in_name=bagfile_in,
                                              bagfile_out_name=bagfile_out,
                                              cfg=cfg_fn,
                                              verbose=verbose,
                                              use_header_timestamp=False,
                                              ignore_new_topic_name=True,
                                              stddev_pos=0.0,
                                              interp_type=interp_type,
                                              min_dt=min_dt)

        if not os.path.isfile(fn_gt_ranges) or IDs_found is None:
            # 3) extract all measurements from the clean bagfile
            res, IDs_found = RelPose_ROSBag2CSV.extract_to_one(bagfile_name=bagfile_out,
                                                    cfg=cfg_fn,
                                                    fn=fn_gt_ranges,
                                                    result_dir=result_dir,
                                                    verbose=verbose)

        # 4) evaluate the ranges
        cfg = AssociateRelPoseCfg(ID1=None,
                                  ID2=None,
                                  relative_timestamps=False,
                                  max_difference=10 ** -4,
                                  subsample=0,
                                  verbose=verbose,
                                  remove_outliers=remove_outliers,
                                  pose_error_type=pose_error_type,
                                  range_error_val=0,
                                  label_timestamp='t',
                                  label_ID1='ID1',
                                  label_ID2='ID2',
                                  label_range='range')

        if max_range is not None:
            cfg.max_range_error = max_range
            cfg.max_range = max_range
        if max_angle is not None:
            cfg.max_angle = max_angle

        if IDs_found is not None:
            # remove IDs from Sensor_ID_arr, if they are not part of the bagfile!
            IDs = list()
            for ID in Sensor_ID_arr:
                if ID in IDs_found:
                    IDs.append(ID)
                elif verbose:
                    print("* Sensor ID= " + str(ID) + "was not found in the rosbag!" )
            Sensor_ID_arr = IDs

        eval = RelPoseMeasEvaluation(fn_gt=fn_gt_ranges,
                                     fn_est=fn_meas_ranges,
                                     ID1_arr=Sensor_ID_arr,
                                     ID2_arr=list(set(list(Sensor_ID_arr + Object_ID_arr))),
                                     cfg=cfg,
                                     result_dir=str(result_dir + '/eval/'),
                                     prefix='',
                                     save_plot=save_plot,
                                     show_plot=show_plot,
                                     save_statistics=True,
                                     plot_timestamps=extra_plots,
                                     plot_ranges=extra_plots,
                                     plot_angles=extra_plots,
                                     plot_ranges_sorted=extra_plots,
                                     plot_range_error=extra_plots,
                                     plot_angle_error=extra_plots,
                                     plot_range_histogram=True,
                                     plot_angle_histogram=True,
                                     filter_histogram=filter_histogram,
                                     plot_position_err=True,
                                     plot_pose_err=True,
                                     plot_pose=True,
                                     verbose=verbose
                                     )
        pass  # DONE


def main():
    # RelPoseMeasEvaluationTool.py --bagfile  ./test/sample_data/T1_A3_loiter_2m_2023-08-31-20-58-20.bag --cfg ./test/sample_data/config.yaml
    parser = argparse.ArgumentParser(
        description='RelPoseMeasEvaluationTool: evaluation the measured relative poses')
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--bagfile', help='input bag file', default="not specified")
    parser.add_argument('--cfg',
                        help='YAML configuration file describing the setup: ' +
                             '{sensor_positions:{<id>:[x,y,z], ...}, sensor_orientations:{<id>:[w,x,y,z], ...}, ' +
                             'relpose_topics:{<id>:<topic_name>, ...}, true_pose_topics:{<id>:<topic_name>, ...}',
                        default="config.yaml", required=True)
    parser.add_argument('--save_plot', help='saves all plots in the result_dir', action='store_true', default=False)
    parser.add_argument('--show_plot',
                        help='blocks the evaluation, for inspecting individual plots, continuous after closing',
                        action='store_true', default=False)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--extra_plots', help='plots: timestamps, ranges + angles (sorted + unsorted + error),  ',
                        action='store_true', default=False)
    parser.add_argument('--keep_outliers', help='do not apply the max. thresholds on the error', action='store_true',
                        default=False)
    parser.add_argument('--filter_histogram',
                        help='filters the error histogram, such that the fitted normal distribution is computed on the best bins only',
                        action='store_true', default=False)
    parser.add_argument('--max_range', help='max. range that classifies them as outlier (0 disables feature). ',
                        default='0')
    parser.add_argument('--max_angle', help='max. range that classifies them as outlier (0 disables feature)',
                        default='0')
    parser.add_argument('--interpolation_type', help='Trajectory interpolation type',
                        choices=TrajectoryInterpolationType.list(),
                        default=str(TrajectoryInterpolationType.linear))
    parser.add_argument('--min_dt',
                        help='temporal displacement of cubic spline control points',
                        default=0.05)
    parser.add_argument('--pose_error_type', help='Covariance perturbation type (space) of relative pose measurements',
                        choices=EstimationErrorType.list(),
                        default=str(EstimationErrorType.type5))

    tp_start = time.time()
    args = parser.parse_args()

    RelPoseMeasEvaluationTool.evaluate(bagfile_in=args.bagfile,
                                       cfg_fn=args.cfg,
                                       result_dir=args.result_dir,
                                       save_plot=args.save_plot,
                                       show_plot=args.show_plot,
                                       verbose=args.verbose,
                                       max_range=float(args.max_range),
                                       max_angle=float(args.max_angle),
                                       extra_plots=args.extra_plots,
                                       filter_histogram=args.filter_histogram,
                                       remove_outliers=not args.keep_outliers,
                                       interp_type=TrajectoryInterpolationType(args.interpolation_type),
                                       min_dt=float(args.min_dt),
                                       pose_error_type=EstimationErrorType(args.pose_error_type)
                                       )
    pass
    print(" ")
    print("finished after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
