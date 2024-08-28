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
import unittest

from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.TrajectoryPlotter import *
from cnspy_relative_pose_evaluation.BsplineSE3 import *
SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')

class BsplineSE3_Test(unittest.TestCase):

    @staticmethod
    def test_interp():
        traj_sub = Trajectory(fn=SAMPLE_DATA_DIR+'/pose-sub.csv')
        traj = Trajectory(fn=SAMPLE_DATA_DIR+'/pose-raw.csv')

        spline = BsplineSE3(traj=traj_sub)

        traj_ = spline.get_trajectory(traj.t_vec)

        fig = plt.figure(figsize=(20, 15), dpi=int(300))
        ax = fig.add_subplot(111, projection='3d')
        traj.plot_3D(fig=fig, ax=ax, cfg=TrajectoryPlotConfig(close_figure=False, show=False), num_markers=0)
        traj_sub.plot_3D(fig=fig, ax=ax, cfg=TrajectoryPlotConfig(close_figure=False, show=False), num_markers=0)
        traj_.plot_3D(fig=fig, ax=ax, cfg=TrajectoryPlotConfig(close_figure=False, show=True), num_markers=0)

if __name__ == "__main__":
    unittest.main()