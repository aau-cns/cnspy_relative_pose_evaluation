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

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')
RES_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_results')


class RelPoseMeasEvaluation_Test(unittest.TestCase):
    def test_plot_NEES(self):
        dict_fn_gt = dict({'GT': SAMPLE_DATA_DIR + '/synth/meas_B1B2_N1000_ref_np-0_nR-0.csv',
                           'CUBIC': SAMPLE_DATA_DIR + '/synth/meas_B1B2_N970_SUB10_CUBIC_np-0.01_nR-0.01.csv',
                           'LINEAR': SAMPLE_DATA_DIR + '/synth/meas_B1B2_N991_SUB10_LINEAR_np-0.01_nR-0.01.csv'
                           })
        for key,fn_gt in dict_fn_gt.items():
            err_types = list([EstimationErrorType.type1, EstimationErrorType.type2, EstimationErrorType.type5])
            #err_types = list([EstimationErrorType.type1])
            for err_type in err_types:
                fn_est = SAMPLE_DATA_DIR + '/synth/meas_B1B2_N1000_'+str(err_type)+'_np-0.1_nR-0.1.csv'

                RelPoseMeasEvaluation(fn_gt, fn_est,
                                      result_dir=RES_DATA_DIR + '/RPME/REF_'+ str(key) + '/' + str(err_type) + '/',
                                      cfg=AssociateRelPoseCfg(pose_error_type=err_type),
                                      save_plot=True,
                                      show_plot=False,
                                      filter_histogram=False,
                                      plot_timestamps=False,
                                      plot_ranges=False,
                                      plot_angles=False,
                                      plot_ranges_sorted=False,
                                      plot_range_error=False,
                                      plot_angle_error=False,
                                      plot_range_histogram=True,
                                      plot_angle_histogram=True,
                                      plot_position_err=False,
                                      plot_pose_err=False,
                                      plot_pose=False,
                                      plot_NEES=False,
                                      verbose=True,
                                      save_statistics=True)


    def test_with_noise(self):
        fn_gt = SAMPLE_DATA_DIR + '/synth/meas_B1B2_N1000_ref_np-0_nR-0.csv'

        err_types = list([EstimationErrorType.type1, EstimationErrorType.type2, EstimationErrorType.type5])
        # err_types = list([EstimationErrorType.type1])
        for err_type in err_types:
            fn_est = SAMPLE_DATA_DIR + '/synth/meas_B1B2_N1000_'+str(err_type)+'_np-0.1_nR-0.1.csv'

            RelPoseMeasEvaluation(fn_gt, fn_est,
                                  result_dir=RES_DATA_DIR + '/RPME/REF_GT/'+ str(err_type) + '/',
                                  cfg=AssociateRelPoseCfg(pose_error_type=err_type),
                                  save_plot=True,
                                  show_plot=False,
                                  filter_histogram=False,
                                  plot_timestamps=False,
                                  plot_ranges=True,
                                  plot_angles=True,
                                  plot_ranges_sorted=True,
                                  plot_range_error=True,
                                  plot_angle_error=True,
                                  plot_range_histogram=True,
                                  plot_angle_histogram=True,
                                  plot_position_err=True,
                                  plot_pose_err=True,
                                  plot_pose=True,
                                  plot_NEES=True,
                                  verbose=True,
                                  save_statistics=True)

        self.assertEqual(True, True)  # add assertion here


if __name__ == '__main__':
    unittest.main()
