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


SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')
RES_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_results')


class EvaluationTrail:

    @staticmethod
    def test_eval():
        sequence = 'static_test'
        bagfile_in = str(SAMPLE_DATA_DIR + '/' + sequence + '.bag')
        cfg_file = str(SAMPLE_DATA_DIR + '/config.yaml')
        ROSBag_Poses2RelPose.extract(bagfile_name=bagfile_in,
                                     cfg=cfg_file,
                                     verbose=True,
                                     filename=None,
                                     result_dir=RES_DATA_DIR)


if __name__ == "__main__":
     EvaluationTrail.test_eval()