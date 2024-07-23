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
from cnspy_relative_pose_evaluation.ROSBag_Poses2RelPoses import *
from cnspy_relative_pose_evaluation.ROSBag_TrueRelPoses import *
from cnspy_relative_pose_evaluation.RelPose_ROSBag2CSV import *
from cnspy_relative_pose_evaluation.RelPoseMeasEvaluationTool  import *

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')
RES_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_results')


class EvaluationTrail:

    @staticmethod
    def test_ROSBag_Poses2RelPose():
        sequence = 'static_test'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        ROSBag_Poses2RelPose.extract(bagfile_name=bagfile_in,
                                     cfg=cfg_file,
                                     verbose=True,
                                     filename=None,
                                     result_dir=RES_DATA_DIR)

    @staticmethod
    def test_ROSBag_TrueRelPoses():
        sequence = 'static_test'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        bagfile_out = str(RES_DATA_DIR + '/' + sequence + 'gt.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        ROSBag_TrueRelPoses.extract(bagfile_in_name=bagfile_in,
                                    bagfile_out_name=bagfile_out,
                                     cfg=cfg_file,
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
    def test_RelPoseEval():
        sequence = 'two_spiral_2to3m'
        sequence = 'static_test'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        RelPoseMeasEvaluationTool.evaluate(bagfile_in=bagfile_in,
                                           cfg=cfg_file,
                                           result_dir=RES_DATA_DIR + "/" + sequence,
                                           save_plot=True,
                                           show_plot=False,
                                           verbose=True,
                                           ID_arr=[0,1,2])


if __name__ == "__main__":
     #EvaluationTrail.test_ROSBag_Poses2RelPose()
     #EvaluationTrail.test_ROSBag_TrueRelPoses()
     #EvaluationTrail.test_RelPose_ROSbag2CSV()

     EvaluationTrail.test_RelPoseEval()

