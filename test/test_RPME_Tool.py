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

from cnspy_relative_pose_evaluation.RelPoseMeasEvaluation import *
from cnspy_relative_pose_evaluation.RelPoseMeasEvaluationTool import RelPoseMeasEvaluationTool
from cnspy_trajectory.BsplineSE3 import TrajectoryInterpolationType

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')
RES_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_results')


class RelPoseMeasEvaluation_Test(unittest.TestCase):
    def test_dataset_eval(self):
        DATASET_DIR= "/home/jungr/workspace/datasets/uvdar_dataset/eval/"
        RES_DIR= "/home/jungr/workspace/datasets/uvdar_dataset/eval/"

        RelPoseMeasEvaluationTool.evaluate(bagfile_in=SAMPLE_DATA_DIR + '/mukisano_6/output.bag',
                                           cfg_fn=SAMPLE_DATA_DIR + '/mukisano_6/config.yaml',
                                           result_dir=RES_DATA_DIR + '/mukisano_6',
                                           save_plot=True,
                                           show_plot=True,
                                           verbose=True,
                                           max_range=float(50),
                                           max_angle=None,
                                           extra_plots=True,
                                           filter_histogram=False,
                                           remove_outliers=False,
                                           ID_arr=[0],
                                           interp_type=TrajectoryInterpolationType.linear,
                                           min_dt=float(0.01),
                                           pose_error_type=EstimationErrorType.type1
                                           )



if __name__ == '__main__':
    unittest.main()
