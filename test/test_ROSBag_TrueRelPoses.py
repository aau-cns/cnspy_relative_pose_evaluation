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
########################################################################################################################
import unittest
import os

from cnspy_relative_pose_evaluation.ROSBag_TrueRelPoses import *
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_trajectory.PlotLineStyle import PlotLineStyle
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.TrajectoryPlotter import TrajectoryPlotter
from cnspy_trajectory_evaluation.TrajectoryAlignmentTypes import TrajectoryAlignmentTypes
from cnspy_trajectory_evaluation.TrajectoryEvaluation import TrajectoryEvaluation

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')
RES_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_results')




class ROSBag_TrueRelPoses_Test(unittest.TestCase):


    def test_interpolation(self):
        sequence = "static_test"
        #sequence = 'two_spiral_2to3m'
        cfg_fn = str(SAMPLE_DATA_DIR + '/config.yaml')
        result_dir = RES_DATA_DIR + "/" + sequence
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')

        bag = rosbag.Bag(bagfile_in)
        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)
        num_messages = num_messages = info_dict['messages']
        verbose = True
        ignore_new_topic_name = True
        interp_type = TrajectoryInterpolationType.cubic
        min_dt = 0.1
        dict_cfg = ROSBag_TrueRelPoses.load_dict_cfg(cfg_fn=cfg_fn,
                                                     ignore_new_topic_name=ignore_new_topic_name,
                                                     verbose=verbose)
        round_decimals = 4
        dict_bsplines, dict_history = ROSBag_TrueRelPoses.load_dict_splines(bag, dict_cfg, interp_type, min_dt,
                                                                              num_messages, round_decimals)

        for key, bspline in dict_bsplines.items():
            t_arr = dict_history[key].t_vec

            dict_pose = dict_history[key]
            if dict_pose and bspline and len(dict_pose.t_vec):
                traj_in = Trajectory(hist_pose=dict_pose)
                traj_out = bspline.get_trajectory(t_arr=traj_in.t_vec,interp_type=interp_type)

                cfg = TrajectoryPlotConfig(show=True,
                                           close_figure=False,
                                           save_fn=result_dir + '/bspline_test/' + str(interp_type) + "/" + key.replace("/", "-") + "_mindt_" + str(min_dt) + ".png")
                TrajectoryPlotter.multi_plot_3D([traj_in, traj_out],
                                                cfg=cfg,
                                                name_list=['gt-'+key, 'interp-'+key],
                                                ls=PlotLineStyle(linestyle='--'))


                TE = TrajectoryEvaluation(traj_gt=traj_out,
                                          traj_est=traj_in,
                                          result_dir=result_dir + '/bspline_test/' + str(interp_type) + "/" + key.replace("/", "-") + "_mindt_" + str(min_dt) + "/",
                                          alignment_type=TrajectoryAlignmentTypes.none,
                                          plot=True,
                                          est_err_type=EstimationErrorType.type5,
                                          verbose=True)


    def test_extract(self):
        #sequence = "static_test"
        sequence = 'two_spiral_2to3m'

        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        cfg_fn = str(SAMPLE_DATA_DIR + '/config.yaml')
        result_dir = RES_DATA_DIR + "/" + sequence

        bagfile_out = str(result_dir +  '/true-ranges.bag')
        res = ROSBag_TrueRelPoses.extract(bagfile_in_name=bagfile_in,
                                          bagfile_out_name=bagfile_out,
                                          cfg=cfg_fn,
                                          verbose=True,
                                          use_header_timestamp=False,
                                          ignore_new_topic_name=True,
                                          stddev_pos=0.0,
                                          interp_type=TrajectoryInterpolationType.cubic,
                                          min_dt=0.05)

        self.assertEqual(True, res)  # add as

        res = ROSBag_TrueRelPoses.extract(bagfile_in_name=bagfile_in,
                                          bagfile_out_name=bagfile_out,
                                          cfg=cfg_fn,
                                          verbose=True,
                                          use_header_timestamp=False,
                                          ignore_new_topic_name=True,
                                          stddev_pos=0.0,
                                          interp_type=TrajectoryInterpolationType.linear,
                                          min_dt=0.05)

        self.assertEqual(True, res)  # add as
        pass


    def test_add_noise(self):
        sequence = "static_test"
        #sequence = 'two_spiral_2to3m'

        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        cfg_fn = str(SAMPLE_DATA_DIR + '/config.yaml')
        result_dir = RES_DATA_DIR + "/" + sequence + "/"

        bagfile_out = str(result_dir +  '/noisy-ranges-type1.bag')
        res = ROSBag_TrueRelPoses.extract(bagfile_in_name=bagfile_in,
                                          bagfile_out_name=bagfile_out,
                                          cfg=cfg_fn,
                                          verbose=True,
                                          use_header_timestamp=False,
                                          ignore_new_topic_name=True,
                                          stddev_pos=0.1,
                                          stddev_or=0.1,
                                          pose_error_type=EstimationErrorType.type1,
                                          interp_type=TrajectoryInterpolationType.cubic,
                                          min_dt=0.05)

        bagfile_out = str(result_dir +  '/noisy-ranges-type5.bag')
        res = ROSBag_TrueRelPoses.extract(bagfile_in_name=bagfile_in,
                                          bagfile_out_name=bagfile_out,
                                          cfg=cfg_fn,
                                          verbose=True,
                                          use_header_timestamp=False,
                                          ignore_new_topic_name=True,
                                          stddev_pos=0.1,
                                          stddev_or=0.1,
                                          pose_error_type=EstimationErrorType.type5,
                                          interp_type=TrajectoryInterpolationType.cubic,
                                          min_dt=0.05)

if __name__ == '__main__':
    unittest.main()
