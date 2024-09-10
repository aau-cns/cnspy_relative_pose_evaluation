#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2020, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
#
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
# Requirements:
# numpy, matplotlib
########################################################################################################################
import os
import math
import time
from sys import version_info
from matplotlib import pyplot as plt
import numpy as np
import pandas as pandas
from spatialmath import UnitQuaternion

from cnspy_csv2dataframe.PosOrientWithCov2DataFrame import PosOrientWithCov2DataFrame
from cnspy_numpy_utils.numpy_statistics import *
from cnspy_timestamp_association.TimestampAssociation import TimestampAssociation
from cnspy_trajectory.PlotLineStyle import PlotLineStyle
from cnspy_ranging_evaluation.AssociateRanges import AssociateRanges, AssociateRangesCfg
from cnspy_trajectory.SpatialConverter import SpatialConverter

from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryEstimated import TrajectoryEstimated
from cnspy_trajectory.TrajectoryError import TrajectoryError
from cnspy_trajectory.TrajectoryEstimationError import TrajectoryEstimationError
from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.TrajectoryErrorType import TrajectoryErrorType

from cnspy_trajectory_evaluation.EstimationTrajectoryError import EstimationTrajectoryError
from cnspy_trajectory_evaluation.AbsoluteTrajectoryError import AbsoluteTrajectoryError
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_spatial_csv_formats.ErrorRepresentationType import ErrorRepresentationType
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType
from cnspy_spatial_csv_formats.CSVSpatialFormat import CSVSpatialFormat
from cnspy_trajectory_evaluation.TrajectoryPosOrientNEES import TrajectoryPosOrientNEES
from cnspy_trajectory_evaluation.TrajectoryPoseNEES import TrajectoryPoseNEES


class AssociateRelPoseCfg(AssociateRangesCfg):
    label_angle = 'angle'
    max_angle = np.pi * 2
    angle_error_val = 0
    pose_error_type = EstimationErrorType.type5

    def __init__(self, ID1=None, ID2=None,
                 relative_timestamps=False,
                 max_difference: float = 0.02,
                 subsample: int = 0,
                 verbose=False,
                 remove_outliers=False,
                 max_range: float = 30,
                 range_error_val: float = 0,
                 label_timestamp: str = 't',
                 label_ID1: str = 'ID1',
                 label_ID2: str = 'ID2',
                 label_range: str = 'range',
                 label_angle: str = 'angle',
                 pose_error_type: EstimationErrorType = EstimationErrorType.type5):
        self.label_timestame = label_timestamp
        self.label_ID1 = label_ID1
        self.label_ID2 = label_ID2
        self.label_range = label_range
        self.label_angle = label_angle
        self.ID1 = ID1
        self.ID2 = ID2
        self.relative_timestamps = relative_timestamps
        self.max_difference = max_difference
        self.subsample = subsample
        self.verbose = verbose
        self.remove_outliers = remove_outliers
        self.max_range = max_range
        self.range_error_val = range_error_val
        self.pose_error_type = pose_error_type


class AssociateRelPoses(AssociateRanges):
    csv_df_gt = None
    csv_df_est = None

    data_frame_gt_matched = None
    data_frame_est_matched = None

    matches_est_gt = None  # list of tuples containing [(idx_est, idx_gt), ...]

    cfg = AssociateRelPoseCfg()
    data_loaded = False
    perc_outliers = 0.0

    traj_err = None  # TrajectoryEstimationError()
    traj_est = None
    traj_gt = None
    traj_nees = None  # TrajectoryPosOrientNEES

    def __init__(self, fn_gt, fn_est, cfg):
        assert (isinstance(cfg, AssociateRelPoseCfg))
        # AssociateRanges.__init__(self)
        self.cfg = cfg
        self.load(fn_gt, fn_est, cfg)
        self.compute_error()

    def load(self, fn_gt, fn_est, cfg):
        assert (isinstance(cfg, AssociateRelPoseCfg))
        assert (os.path.exists(fn_gt)), str("Path to fn_gt does not exist!:" + str(fn_gt))
        assert (os.path.exists((fn_est))), str("Path to fn_est does not exist!:" + str(fn_est))

        self.csv_df_gt = pandas.read_csv(fn_gt, sep='\s+|\,', comment='#', engine='python')
        self.csv_df_est = pandas.read_csv(fn_est, sep='\s+|\,', comment='#', engine='python')

        if cfg.ID1 is not None:
            self.csv_df_gt = (self.csv_df_gt.loc[self.csv_df_gt[cfg.label_ID1] == cfg.ID1])
            self.csv_df_est = (self.csv_df_est.loc[self.csv_df_est[cfg.label_ID1] == cfg.ID1])

            if len(self.csv_df_gt) == 0 or len(self.csv_df_est) == 0:
                print("ID1=" + str(cfg.ID1) + " not found")
                return

        if cfg.ID2 is not None:
            self.csv_df_gt = (self.csv_df_gt.loc[self.csv_df_gt[cfg.label_ID2] == cfg.ID2])
            self.csv_df_est = (self.csv_df_est.loc[self.csv_df_est[cfg.label_ID2] == cfg.ID2])
            if len(self.csv_df_gt) == 0 or len(self.csv_df_est) == 0:
                print("ID2=" + str(cfg.ID2) + " not found")
                return

        # compute range
        if 'range' not in self.csv_df_est:
            range_est = np.linalg.norm(self.csv_df_est[['tx', 'ty', 'tz']].to_numpy(), axis=1)
            self.csv_df_est[cfg.label_range] = range_est
        if 'range' not in self.csv_df_gt:
            range_gt = np.linalg.norm(self.csv_df_gt[['tx', 'ty', 'tz']].to_numpy(), axis=1)
            self.csv_df_gt[cfg.label_range] = range_gt
        # compute angle
        if 'angle' not in self.csv_df_est:
            self.csv_df_est[cfg.label_angle] = AssociateRelPoses.quat2ang_arr(
                self.csv_df_est[['qw', 'qx', 'qy', 'qz']].to_numpy())
        if 'angle' not in self.csv_df_gt:
            self.csv_df_gt[cfg.label_angle] = AssociateRelPoses.quat2ang_arr(
                self.csv_df_gt[['qw', 'qx', 'qy', 'qz']].to_numpy())

        if cfg.remove_outliers:
            N = len(self.csv_df_est.index)
            if cfg.max_range > 0:
                indices1 = np.nonzero(self.csv_df_est[cfg.label_range] < cfg.max_range)
            else:
                indices1 = np.array(range(1, N)) # all are inliers
            indices2 = np.nonzero(self.csv_df_est[cfg.label_range] > 0)
            if cfg.max_angle > 0:
                indices3 = np.nonzero(abs(self.csv_df_est[cfg.label_angle]) < cfg.max_angle)
            else:
                indices3 = np.array(range(1, N)) # all are inliers

            idx_inliers = np.intersect1d(np.intersect1d(indices1, indices2), indices3)
            num_outliers = N - len(idx_inliers)
            self.perc_outliers = 100.0 * (num_outliers / max(1, N))
            if len(idx_inliers):
                self.csv_df_est = AssociateRanges.sample_DataFrame(self.csv_df_est, idx_inliers)
                print('AssociateRelPoses.load(): [%d] outliers (%.1f %%) removed!' % (num_outliers, self.perc_outliers))
        else:
            indices = ((self.csv_df_est[cfg.label_range]) < 0)
            self.csv_df_est.loc[indices, cfg.label_range] = cfg.range_error_val
            if cfg.max_range > 0:
                indices = (self.csv_df_est[cfg.label_range] > cfg.max_range)
                self.csv_df_est.loc[indices, cfg.label_range] = cfg.range_error_val
            if cfg.max_angle > 0:
                indices = (self.csv_df_est[cfg.label_angle] > cfg.max_angle)
                self.csv_df_est.loc[indices, cfg.label_angle] = cfg.angle_error_val
        if cfg.subsample > 1:
            subsample = round(cfg.subsample, 0)
            self.csv_df_gt = AssociateRanges.subsample_DataFrame(df=self.csv_df_gt, step=subsample, verbose=cfg.verbose)
            self.csv_df_est = AssociateRanges.subsample_DataFrame(df=self.csv_df_est, step=subsample,
                                                                  verbose=cfg.verbose)

        if version_info[0] < 3:
            t_vec_gt = self.csv_df_gt.as_matrix([cfg.label_timestamp])
            t_vec_est = self.csv_df_est.as_matrix([cfg.label_timestamp])
            t_zero = min(t_vec_gt[0], t_vec_est[0])

            if len(t_vec_gt) == 0 or len(t_vec_est) == 0:
                print("empty")
                return

            if cfg.relative_timestamps:
                self.csv_df_gt[[cfg.label_timestamp]] = self.csv_df_gt[[cfg.label_timestamp]] - t_zero
                self.csv_df_est[[cfg.label_timestamp]] = self.csv_df_est[[cfg.label_timestamp]] - t_zero
        else:
            # FIX(scm): for newer versions as_matrix is deprecated, using to_numpy instead
            # from https://stackoverflow.com/questions/60164560/attributeerror-series-object-has-no-attribute-as-matrix-why-is-it-error
            t_vec_gt = self.csv_df_gt[[cfg.label_timestamp]].to_numpy()
            t_vec_est = self.csv_df_est[[cfg.label_timestamp]].to_numpy()
            if len(t_vec_gt) == 0 or len(t_vec_est) == 0:
                print("empty")
                return

            t_zero = min(t_vec_gt[0], t_vec_est[0])
            if cfg.relative_timestamps:
                self.csv_df_gt[[cfg.label_timestamp]] = self.csv_df_gt[[cfg.label_timestamp]] - t_zero
                self.csv_df_est[[cfg.label_timestamp]] = self.csv_df_est[[cfg.label_timestamp]] - t_zero

        if cfg.relative_timestamps:
            # only relative time stamps:
            t_vec_gt = t_vec_gt - t_zero
            t_vec_est = t_vec_est - t_zero

        idx_est, idx_gt, t_est_matched, t_gt_matched = TimestampAssociation.associate_timestamps(
            t_vec_est,
            t_vec_gt,
            max_difference=cfg.max_difference,
            round_decimals=6,
            unique_timestamps=True)

        self.data_frame_est_matched = self.csv_df_est.iloc[idx_est, :]
        self.data_frame_gt_matched = self.csv_df_gt.iloc[idx_gt, :]
        self.matches_est_gt = zip(idx_est, idx_gt)

        if cfg.verbose:
            print("AssociateRelPoses(): {} timestamps associated.".format(len(idx_est)))

        self.data_loaded = True
        # using zip() and * operator to
        # perform Unzipping
        # res = list(zip(*test_list))

    def num_samples(self) -> int:
        if self.data_loaded:
            return len(self.data_frame_est_matched.index)
        else:
            return 0

    def compute_error(self):
        if not self.data_loaded:
            return None
        assert version_info[0] >= 3, "Unsupported dataframe version..."

        Sigma_p_vec, Sigma_R_vec = PosOrientWithCov2DataFrame.cov_from_DataFrame(self.data_frame_est_matched)
        # Note: Trajectory uses the weired HTMQ quaternion conventions, with real part last...
        self.traj_est = TrajectoryEstimated(t_vec=self.data_frame_est_matched['t'].to_numpy(),
                                            p_vec=self.data_frame_est_matched[['tx', 'ty', 'tz']].to_numpy(),
                                            q_vec=self.data_frame_est_matched[['qx', 'qy', 'qz', 'qw']].to_numpy(),
                                            Sigma_p_vec=Sigma_p_vec, Sigma_R_vec=Sigma_R_vec)

        if self.cfg.pose_error_type == EstimationErrorType.type5 or self.cfg.pose_error_type == EstimationErrorType.type6:
            err_rep_type = ErrorRepresentationType.theta_so3
        elif self.cfg.pose_error_type == EstimationErrorType.type1 or self.cfg.pose_error_type == EstimationErrorType.type2:
            err_rep_type = ErrorRepresentationType.tau_se3
        else:
            assert False, "unsupported estimation error type: either 1,2,5,6! Got" + str(self.cfg.pose_error_type)

        est_fmt = CSVSpatialFormat(fmt_type=CSVSpatialFormatType.PosOrientWithCov,
                                   est_err_type=self.cfg.pose_error_type,
                                   err_rep_type=err_rep_type)
        self.traj_est.set_format(est_fmt)
        self.traj_gt = Trajectory(t_vec=self.data_frame_gt_matched['t'].to_numpy(),
                                  p_vec=self.data_frame_gt_matched[['tx', 'ty', 'tz']].to_numpy(),
                                  q_vec=self.data_frame_gt_matched[['qx', 'qy', 'qz', 'qw']].to_numpy())

        self.traj_err = AbsoluteTrajectoryError.compute_trajectory_error(traj_est=self.traj_est, traj_gt=self.traj_gt,
                                                                         traj_err_type=TrajectoryErrorType(
                                                                             est_fmt.estimation_error_type))

        # 1) traj_err is converted to error definition, e.g. on the se(3) or on so(3)x R3
        traj_est_err = TrajectoryEstimationError(t_vec=self.traj_err.t_vec, p_vec=self.traj_err.p_vec,
                                                 q_vec=self.traj_err.q_vec,
                                                 est_err_type=est_fmt.estimation_error_type,
                                                 err_rep_type=est_fmt.rotation_error_representation)
        # 2) the NEES should be computed for R(3)xso(3) or  se(3):
        if self.cfg.pose_error_type == EstimationErrorType.type5 or self.cfg.pose_error_type == EstimationErrorType.type6:
            self.traj_nees = TrajectoryPosOrientNEES(traj_est=self.traj_est, traj_err=traj_est_err)
        else:
            self.traj_nees = TrajectoryPoseNEES(traj_est=self.traj_est, traj_err=traj_est_err)
        pass

    def avg_NEES(self) -> dict:
        dict_nees = dict()
        if isinstance(self.traj_nees, TrajectoryPosOrientNEES):
            p, R = self.traj_nees.get_avg_NEES()
            dict_nees['nees_p'] = p
            dict_nees['nees_R'] = R
        elif isinstance(self.traj_nees, TrajectoryPoseNEES):
            dict_nees['nees_T'] = self.traj_nees.get_avg_NEES()

        return dict_nees

    def plot_position_err(self, cfg=TrajectoryPlotConfig(), fig=None, plot_NEES=True):
        if not self.data_loaded:
            return

        if plot_NEES:
            ax1 = fig.add_subplot(3, 1, 1)
            ax2 = fig.add_subplot(3, 1, 2)
            ax3 = fig.add_subplot(3, 1, 3)
        else:
            ax1 = fig.add_subplot(2, 1, 1)
            ax2 = fig.add_subplot(2, 1, 2)
            ax3 = None
        TrajectoryError.plot_position_err(traj_est=self.traj_est,
                                          traj_err=self.traj_err,
                                          cfg=cfg, fig=fig,
                                          axes=[ax1, ax2])

        if self.traj_est.Sigma_p_vec is not None or self.traj_est.Sigma_T_vec is not None:
            self.traj_est.ax_plot_p_sigma(ax2, cfg=cfg, colors=['darkred', 'darkgreen', 'darkblue'],
                                          ls=PlotLineStyle(linewidth=0.5, linestyle='-.'))

        if plot_NEES:
            if isinstance(self.traj_nees, TrajectoryPosOrientNEES):
                self.traj_nees.plot_NEES_p(ax1=ax3, relative_time=cfg.relative_time)
                ax3.set_yscale('log')
                ax3.grid()
                return fig, ax1, ax2, ax3
            else:
                self.traj_nees.plot_NEES(ax1=ax3, relative_time=cfg.relative_time)
                ax3.set_yscale('log')
                ax3.grid()
                return fig, ax1, ax2, ax3

        else:
            return fig, ax1, ax2, ax3

    def plot_traj_err(self, cfg=TrajectoryPlotConfig(), fig=None, plot_NEES=True):
        if not self.data_loaded:
            return

        if plot_NEES:
            ax1 = fig.add_subplot(3, 2, 1)
            ax2 = fig.add_subplot(3, 2, 3)
            ax3 = fig.add_subplot(3, 2, 2)
            ax4 = fig.add_subplot(3, 2, 4)
            ax5 = fig.add_subplot(3, 2, 5)
            ax6 = fig.add_subplot(3, 2, 6)
        else:
            ax1 = fig.add_subplot(221)
            ax2 = fig.add_subplot(222)
            ax3 = fig.add_subplot(223)
            ax4 = fig.add_subplot(224)

        TrajectoryError.plot_pose_err(traj_est=self.traj_est, traj_err=self.traj_err,
                                      cfg=cfg, fig=fig,
                                      angles=True, plot_rpy=True, axes=[ax1, ax2, ax3, ax4])

        if self.traj_est.Sigma_R_vec is not None or self.traj_est.Sigma_T_vec is not None:
            self.traj_est.ax_plot_p_sigma(ax2, cfg=cfg, colors=['darkred', 'darkgreen', 'darkblue'],
                                          ls=PlotLineStyle(linewidth=0.5, linestyle='-.'))
            self.traj_est.ax_plot_rpy_sigma(ax4, cfg=cfg, colors=['darkred', 'darkgreen', 'darkblue'],
                                            ls=PlotLineStyle(linewidth=0.5, linestyle='-.'))

        if plot_NEES:
            if isinstance(self.traj_nees, TrajectoryPosOrientNEES):
                self.traj_nees.plot_NEES_p(ax1=ax5, relative_time=cfg.relative_time)
                ax5.set_yscale('log')
                ax5.grid()
                self.traj_nees.plot_NEES_R(ax2=ax6, relative_time=cfg.relative_time)
                ax6.set_yscale('log')
                ax6.grid()
                return fig, ax1, ax2, ax3, ax4, ax5, ax6
            else:
                self.traj_nees.plot_NEES(ax1=ax5, relative_time=cfg.relative_time)
                ax5.set_yscale('log')
                ax5.grid()
                return fig, ax1, ax2, ax3, ax4, ax5, ax6

        else:
            return fig, ax1, ax2, ax3, ax4

    def plot_traj(self, cfg=TrajectoryPlotConfig(), fig=None):
        if not self.data_loaded:
            return

        fig, ax1, ax2, ax3, ax4 = TrajectoryError.plot_pose(traj_est=self.traj_est, traj_gt=self.traj_gt,
                                                            cfg=cfg, fig=fig,
                                                            angles=True, plot_rpy=True)

        return fig, ax1, ax2, ax3, ax4

    @staticmethod
    def quat2ang_arr(q_arr):
        def myfunction(x):
            q = UnitQuaternion(x)
            [angle, vec] = q.angvec()
            return angle

        [rows, cols] = q_arr.shape
        assert (cols == 4);
        return np.apply_along_axis(myfunction, axis=1, arr=q_arr)

    @staticmethod
    def quats2ang_arr(q_arr1, q_arr2):
        def myfunction(x, y):
            q1 = UnitQuaternion(x).unit()
            q2 = UnitQuaternion(y).unit()
            # local error of x
            q = q1.inv() * q2
            [angle, vec] = q.angvec()
            return angle

        [rows1, cols1] = q_arr1.shape
        [rows2, cols2] = q_arr2.shape
        assert (cols1 == 4 and cols2 == 4)
        assert (rows1 == rows2)

        ang_arr = np.zeros([rows1])
        for idx in range(0, rows1):
            ang_arr[idx] = myfunction(q_arr1[idx, :], q_arr2[idx, :])
        return ang_arr

    def plot_angles(self, cfg_dpi=200, cfg_title="angles", sorted=False, fig=None, ax=None,
                    colors=['r', 'g'], labels=['gt', 'est'],
                    ls_vec=[PlotLineStyle(linestyle='-'), PlotLineStyle(linestyle='-.')],
                    save_fn="", result_dir="."):
        if not self.data_loaded:
            return
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)
        if cfg_title:
            ax.set_title(cfg_title)

        if version_info[0] < 3:
            t_vec_gt = self.data_frame_gt_matched.as_matrix([self.cfg.label_timestamp])
            t_vec_est = self.data_frame_est_matched.as_matrix([self.cfg.label_timestamp])
        else:
            t_vec_gt = self.data_frame_gt_matched[[self.cfg.label_timestamp]].to_numpy()
            t_vec_est = self.data_frame_est_matched[[self.cfg.label_timestamp]].to_numpy()

        if version_info[0] < 3:
            r_vec_gt = self.data_frame_gt_matched.as_matrix([self.cfg.label_angle])
            r_vec_est = self.data_frame_est_matched.as_matrix([self.cfg.label_angle])
        else:
            r_vec_gt = self.data_frame_gt_matched[[self.cfg.label_angle]].to_numpy()
            r_vec_est = self.data_frame_est_matched[[self.cfg.label_angle]].to_numpy()

        if not sorted:
            # x_arr = range(len(t_vec_gt))
            AssociateRanges.ax_plot_n_dim(ax, t_vec_gt, r_vec_gt, colors=[colors[0]], labels=[labels[0]], ls=ls_vec[0])
            # x_arr = range(len(t_vec_est))
            AssociateRanges.ax_plot_n_dim(ax, t_vec_est, r_vec_est, colors=[colors[1]], labels=[labels[1]],
                                          ls=ls_vec[1])

            ax.grid()
            ax.set_ylabel('angle')
            ax.set_xlabel('time [s]')
            AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        else:
            gt_indices_sorted = np.argsort(r_vec_gt, axis=0)
            x_arr = range(len(r_vec_gt))
            AssociateRanges.ax_plot_n_dim(ax, x_arr, np.take_along_axis(r_vec_gt, gt_indices_sorted, axis=0),
                                          colors=[colors[0]], labels=[labels[0]], ls=ls_vec[0])
            AssociateRanges.ax_plot_n_dim(ax, x_arr, np.take_along_axis(r_vec_est, gt_indices_sorted, axis=0),
                                          colors=[colors[1]], labels=[labels[1]], ls=ls_vec[1])
            ax.grid()
            ax.set_ylabel('angle')
            ax.set_xlabel('angle sorted index')
            AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        ax.legend()
        return fig, ax

    def compute_angle_error(self, sort=False, remove_outlier=True, max_error=None):
        if not self.data_loaded:
            return
        if version_info[0] < 3:
            t_vec_gt = self.data_frame_gt_matched.as_matrix([self.cfg.label_timestamp])
            t_vec_est = self.data_frame_est_matched.as_matrix([self.cfg.label_timestamp])
        else:
            t_vec_gt = self.data_frame_gt_matched[[self.cfg.label_timestamp]].to_numpy()
            t_vec_est = self.data_frame_est_matched[[self.cfg.label_timestamp]].to_numpy()

        if version_info[0] < 3:
            r_vec_gt = self.data_frame_gt_matched.as_matrix([['qw', 'qx', 'qy', 'qz']])
            r_vec_est = self.data_frame_est_matched.as_matrix([['qw', 'qx', 'qy', 'qz']])
        else:
            r_vec_gt = self.data_frame_gt_matched[['qw', 'qx', 'qy', 'qz']].to_numpy()
            r_vec_est = self.data_frame_est_matched[['qw', 'qx', 'qy', 'qz']].to_numpy()

        if not sort:
            # if r_est = r_true + r_err, then  r_err is an offset or the constant bias (gamma).
            r_vec_err = AssociateRelPoses.quats2ang_arr(r_vec_est, r_vec_gt)
            t_vec = t_vec_gt
            if max_error:
                indices = np.nonzero((abs(r_vec_err) > max_error))
                t_vec = np.delete(t_vec, indices, axis=0, )
                r_vec_err = np.delete(r_vec_err, indices, axis=0)

            return [t_vec, r_vec_err]
        else:
            # if r_est = r_true + r_err, then  r_err is an offset or the constant bias (gamma).
            r_vec_err = AssociateRelPoses.quats2ang_arr(r_vec_est, r_vec_gt)
            x_arr = self.data_frame_gt_matched[self.cfg.label_angle].to_numpy()
            angle_est_arr = self.data_frame_est_matched[self.cfg.label_angle].to_numpy()

            if max_error:
                indices = np.nonzero((abs(r_vec_err) > max_error))
                x_arr = np.delete(x_arr, indices, axis=0, )
                r_vec_err = np.delete(r_vec_err, indices, axis=0)

            gt_indices_sorted = np.argsort(x_arr, axis=0)
            # x_arr = range(len(r_vec_gt))
            t_vec = np.take_along_axis(x_arr, gt_indices_sorted, axis=0)
            r_vec = np.take_along_axis(r_vec_err, gt_indices_sorted, axis=0)
            return [t_vec, r_vec]

    def plot_angle_error(self, cfg_dpi=200, cfg_title="angle", sorted=False, remove_outlier=True, fig=None, ax=None,
                         colors=['r'], labels=['error'],
                         ls_vec=[PlotLineStyle(linestyle='-'), PlotLineStyle(linestyle='-.')],
                         save_fn="", result_dir="."):
        if not self.data_loaded:
            return
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)
        if cfg_title:
            ax.set_title(cfg_title)

        [t_vec, r_vec_err] = self.compute_angle_error(sort=sorted, remove_outlier=remove_outlier)
        if not sorted:
            AssociateRanges.ax_plot_n_dim(ax, t_vec, r_vec_err, colors=[colors[0]], labels=[labels[0]], ls=ls_vec[0])
            ax.grid()
            ax.set_ylabel('angle error (gt-est)')
            ax.set_xlabel('time [s]')
            AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        else:
            AssociateRanges.ax_plot_n_dim(ax, t_vec, r_vec_err,
                                          colors=[colors[0]], labels=[labels[0]], ls=ls_vec[0])

            ax.grid()
            ax.set_ylabel('angle error (gt-est)')
            ax.set_xlabel('angle sorted index')
            AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)

        ax.legend()
        stat = numpy_statistics(vNumpy=np.squeeze(np.asarray(r_vec_err)))
        return fig, ax, stat, r_vec_err

    def plot_angle_error_histogram(self, cfg_dpi=200, fig=None, ax=None,
                                   save_fn="", result_dir=".", max_error=None, filter_histogramm=False,
                                   perc_inliers=0.3,
                                   ID1=None, ID2=None):
        if not self.data_loaded:
            return fig, ax, None, None

        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)

        [t_vec, r_vec_err] = self.compute_angle_error(sort=True, remove_outlier=True, max_error=max_error)
        num_bins = 50
        n, bins, patches = ax.hist(r_vec_err, num_bins, density=True, color='red', alpha=0.75, label='Histogram')

        if len(t_vec) == 0 or len(r_vec_err) == 0:
            return fig, ax, None, None

        if not filter_histogramm:
            # add a 'best fit' line
            # NOTE: angle error is always positive, therefore, we create negative side, and mean is 0:
            stat = numpy_statistics(
                vNumpy=np.squeeze(np.concatenate((np.asarray(r_vec_err), np.asarray(-1.0 * r_vec_err)))))
            # stat = numpy_statistics( vNumpy=np.squeeze(np.asarray(abs(r_vec_err))))
            sigma = max(0.001, stat['std'])  # avoid division by 0
            mu = stat['mean']
            y = ((1 / (np.sqrt(2 * np.pi) * sigma)) *
                 np.exp(-0.5 * (1 / sigma * (bins - mu)) ** 2))

            scaling = len(r_vec_err) / num_bins
            ax.plot(bins, y * scaling, '--', color='blue', alpha=0.75, label='PDF')
            ax.set_ylabel('num. samples normalized')
            ax.set_xlabel('error [rad]')
            ax.set_title(r'Angle Error Histogram ID' + str(ID1) + '-ID' + str(ID2) + ': $\mu$=' + str(
                round(mu, 3)) + ', $\sigma$=' + str(round(sigma, 3)))
            ax.legend()
            return fig, ax, stat, r_vec_err
        else:
            idx_n_sorted = np.argsort(n)

            # assuming a certain percentage as inliers:
            num_best_bins = int(num_bins * (0.5 * perc_inliers))

            # compute the mean about the most frequent values:
            idx_best_bins = idx_n_sorted[-num_best_bins:]
            best_error_vals = bins[idx_best_bins]
            mean_best_errors = np.mean(best_error_vals, axis=0)

            # compute a boundary to remove outliers:
            min_offset_errors = mean_best_errors - np.min(best_error_vals, axis=0)
            max_offset_errors = np.max(best_error_vals, axis=0) - mean_best_errors
            print('mean_best_errors %f' % mean_best_errors)

            # remove outliers
            r_filtered_err = r_vec_err[(r_vec_err > (mean_best_errors - min_offset_errors * 2.0)) &
                                       (r_vec_err < (mean_best_errors + 2.0 * max_offset_errors))]

            # add a 'best fit' line
            # NOTE: angle error is always positive, therefore, we create negative side, and mean is 0:
            stat = numpy_statistics(
                vNumpy=np.squeeze(np.concatenate((np.asarray(r_filtered_err), np.asarray(-1.0 * r_filtered_err)))))
            # stat = numpy_statistics(vNumpy=np.squeeze(np.asarray(abs(r_filtered_err))))
            num_plot_bins = int(num_bins * (perc_inliers))
            n_, bins_, patches_ = ax.hist(r_filtered_err, num_plot_bins, density=True, color='blue', alpha=0.75,
                                          label='Histogram (filtered)')
            sigma = max(0.001, stat['std'])  # avoid division by 0
            mu = stat['mean']
            scaling = 1.0;  # len(r_filtered_err)/num_plot_bins
            y = ((1 / (np.sqrt(2 * np.pi) * sigma)) *
                 np.exp(-0.5 * (1 / sigma * (bins_ - mu)) ** 2))
            ax.plot(bins_, y * scaling, '--', color='green', label='PDF (filtered)')
            ax.set_ylabel('num. samples normalized')
            ax.set_xlabel('error [rad]')
            ax.set_title(r'Angle Error Histogram (filtered) ID' + str(ID1) + '-ID' + str(ID2) + ': $\mu$=' + str(
                round(mu, 3)) + ', $\sigma$=' + str(round(sigma, 3)))
            ax.legend()
            return fig, ax, stat, r_filtered_err
        pass

    def plot_NEES(self, cfg_dpi=200, cfg_title="NEES", fig=None, ax=None,
                  label='',
                  ls_vec=[PlotLineStyle(linestyle='-', linecolor='r'), PlotLineStyle(linestyle='-', linecolor='g')],
                  save_fn="", result_dir=".", relative_time=True):
        if not self.data_loaded:
            return
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg_dpi))
        if ax is None:
            ax = fig.add_subplot(111)
        if cfg_title:
            ax.set_title(cfg_title)

        if isinstance(self.traj_nees, TrajectoryPosOrientNEES):
            self.traj_nees.plot_NEES_p(ax1=ax, relative_time=relative_time, ls=ls_vec[0], plot_intervals=False)
            self.traj_nees.plot_NEES_R(ax2=ax, relative_time=relative_time, ls=ls_vec[1])
            if label:
                ax.set_ylable(label)
            else:
                ax.set_ylabel('NEES rot+pos')
            ax.set_yscale('log')
            ax.grid()
        else:
            self.traj_nees.plot_NEES(ax1=ax, relative_time=relative_time, ls=ls_vec[0])
            if label:
                ax.set_ylable(label)
            else:
                ax.set_ylabel('NEES SE(3)')

            ax.set_yscale('log')
            ax.grid()

        ax.legend()
        # AssociateRanges.show_save_figure(fig=fig, result_dir=result_dir, save_fn=save_fn, show=False)
        return fig, ax
