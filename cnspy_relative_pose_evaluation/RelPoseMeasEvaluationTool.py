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

from std_msgs.msg import Header, Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped

class RelPoseMeasEvaluationTool:
    @staticmethod
    def evaluate(bagfile_in, cfg, result_dir=None, verbose=True, show_plot=True, save_plot=True):
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
        with open(cfg, "r") as yamlfile:
            dict_cfg = yaml.load(yamlfile, Loader=yaml.FullLoader)
            if "sensor_topics" not in dict_cfg:
                print("[sensor_topics] does not exist in fn=" + cfg)
                return False
            if "pose_topic" not in dict_cfg:
                print("[pose_topic] does not exist in fn=" + cfg)
                return False
            print("Successfully read YAML config file.")

        Sensor_ID_arr = []
        topic_list = []
        for key, val in dict_cfg["sensor_topics"].items():
            topic_list.append(val)
        for key, val in dict_cfg["pose_topic"].items():
            topic_list.append(val)
            Sensor_ID_arr.append(int(key))

        if verbose:
            print("* topic_list= " + str(topic_list))
            print("* Sensor_ID_arr= " + str(Sensor_ID_arr))

        #topic_list=['/d01/ranging', '/a01/ranging', '/a02/ranging', '/a03/ranging']
        fn_meas_ranges = str(result_dir + '/all-meas-ranges.csv')
        fn_gt_ranges = str(result_dir + '/all-true-ranges.csv')


        # 1) extract all measurements to CSV
        if not os.path.isfile(fn_meas_ranges):
            res = TWR_ROSbag2CSV.extract_to_one(bagfile_name=bagfile_in,
                                                topic_list=topic_list,
                                                fn=fn_meas_ranges,
                                                result_dir=result_dir,
                                                verbose=verbose)



        bagfile_out = str(result_dir + '/true-ranges.bag')

        # 2) create a clean bag file
        if not os.path.isfile(bagfile_out):
            res = ROSbag_TrueRanges.extract(bagfile_in_name=bagfile_in,
                                            bagfile_out_name=bagfile_out,
                                            topic_pose=dict_cfg["pose_topic"],
                                            cfg=cfg,
                                            verbose=verbose,
                                            stddev_range=0.001)

        if not os.path.isfile(fn_gt_ranges):
            # 3) extract all measurements from the clean bagfile
            res = TWR_ROSbag2CSV.extract_to_one(bagfile_name=bagfile_out,
                                                topic_list=topic_list,
                                                fn=fn_gt_ranges,
                                                result_dir=result_dir,
                                                verbose=verbose)


        # 4) evaluate the ranges
        cfg = AssociateRangesCfg(UWB_ID1=None,
                                 UWB_ID2=None,
                                 relative_timestamps=False,
                                 max_difference=0.03,
                                 subsample=0,
                                 verbose=True,
                                 remove_outliers=False,
                                 max_range=30,
                                 range_error_val=0,
                                 label_timestamp='t',
                                 label_ID1='UWB_ID1',
                                 label_ID2='UWB_ID2',
                                 label_range='range_raw')

        eval = RangeEvaluation(fn_gt=fn_gt_ranges,
                               fn_est=fn_meas_ranges,
                               UWB_ID1_arr=UWB_ID1_arr,
                               UWB_ID2_arr=UWB_ID1_arr,
                               cfg=cfg,
                               result_dir=str(result_dir + '/eval/'),
                               prefix='',
                               save_plot=save_plot,
                               show_plot=show_plot,
                               save_statistics=True,
                               plot_timestamps=True,
                               plot_ranges=True,
                               plot_ranges_sorted=True,
                               plot_error=True,
                               plot_histogram=True,
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
                        help='YAML configuration file describing the setup: {rel_tag_positions, abs_anchor_positions}',
                        default="config.yaml", required=True)
    parser.add_argument('--save_plot', action='store_true', default=True)
    parser.add_argument('--show_plot', action='store_true', default=False)
    parser.add_argument('--verbose', action='store_true', default=True)

    tp_start = time.time()
    args = parser.parse_args()

    RelPoseMeasEvaluationTool.evaluate(bagfile_in=args.bagfile,
                                 cfg=args.cfg,
                                 result_dir=args.result_dir,
                                 save_plot=args.save_plot,
                                 show_plot=args.show_plot,
                                 verbose=args.verbose)
    pass


if __name__ == "__main__":
    main()
    pass