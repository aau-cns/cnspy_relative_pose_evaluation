#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2024, Roland Jung (roland.jung@aau.at), AAU, IST-CNS
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
#
# references:
# * OpenVINS: https://docs.openvins.com/classov__core_1_1BsplineSE3.html
########################################################################################################################
import math
from enum import Enum
import numpy as np

from spatialmath import UnitQuaternion, SO3, SE3, Quaternion, base, quaternion
from spatialmath.base.quaternions import qslerp
import spatialmath.base as smb

from cnspy_ranging_evaluation.HistoryBuffer import HistoryBuffer
from cnspy_trajectory.SpatialConverter import SpatialConverter
from cnspy_trajectory.Trajectory import Trajectory

class InterpolationType(Enum):
    CubicBspline_SE3 = 'CubicBspline_SE3' #  cubic b-spline to ensure C²-continuity
    SPLIT_SE3 = 'SPLIT_SE3' # constant linear and angular velocity, no accelerations

    def __str__(self):
        return self.value

    @staticmethod
    def list():
        return list([str(InterpolationType.CubicBspline_SE3),
                     str(InterpolationType.SPLIT_SE3),
                     str(InterpolationType.BsplineFit_SE3) ])


# TODO: move to trajectory package, create a loader from a Trajectory, HistoryBuffer or DataFrame
class BsplineSE3:

    hist_ctrl_pts = HistoryBuffer()

    def __init__(self, hist_pose=None, traj=None):
        self.hist_ctrl_pts = HistoryBuffer()
        if hist_pose and isinstance(hist_pose, HistoryBuffer):
            self.feed_pose_history(hist_pose=hist_pose, uniform_timestamps=True)
        elif traj and isinstance(traj, Trajectory):
            self.feed_trajectory(traj=traj, uniform_timestamps=True)
        pass

    def get_pose(self, t, type=InterpolationType.CubicBspline_SE3, round_decimals=4) -> SE3:
        if type == InterpolationType.CubicBspline_SE3:
            return self.get_pose_CubicBspline(t, round_decimals)
        elif type == InterpolationType.SPLIT_SE3:
            return self.get_pose_SPLIT(t, round_decimals)
        else:
            return None

    def get_pose_CubicBspline(self, t, round_decimals=4) -> SE3:
        t = round(t, round_decimals)
        [success, T_t0, t0, T_t1, t1, T_t2, t2, T_t3, t3] = self.get_control_poses(t, round_decimals)
        if success and t0 < t1 and t1 < t2 and t2 < t3:
            DT = (t2 - t1)
            # De Boor-Cox matrix scalars for uniform timesteps
            u = round((t - t1) / DT, round_decimals)
            b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u)
            b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u)
            b2 = 1.0 / 6.0 * (u * u * u)
            T0 = np.copy(T_t0.A)
            T1 = np.copy(T_t1.A)
            T2 = np.copy(T_t2.A)
            T3 = np.copy(T_t3.A)
            A0 = smb.trexp(b0 * smb.trlog(np.matmul(smb.trinv(T0),T1), check=False), check=False)
            A1 = smb.trexp(b1 * smb.trlog(np.matmul(smb.trinv(T1),T2), check=False), check=False)
            A2 = smb.trexp(b2 * smb.trlog(np.matmul(smb.trinv(T2),T3), check=False), check=False)
            T_i = np.matmul(T0, np.matmul(A0, np.matmul(A1, A2)))
            return SE3(T_i, check=False)
        return None

    def get_pose_SPLIT(self, t, round_decimals=4) -> SE3:
        t = round(t, round_decimals)
        [success, T_t0, t0, T_t1, t1] = self.get_bounding_poses(t, round_decimals)
        if success:
            assert (t0 <= t1), "Timestamp not ascending!"

            dt = t1 - t0
            dt_i = t - t0
            i = abs(dt_i / dt)

            if dt < 10 ** -3:
                return T_t0
            if i < 10 ** -2:
                return T_t0
            elif i > 1.0 - 10 ** -2:
                return T_t1
            # interpolate between poses:
            q0 = UnitQuaternion(T_t0.R)
            q1 = UnitQuaternion(T_t1.R)
            p0 = T_t0.t
            p1 = T_t1.t

            qi = UnitQuaternion(qslerp(q0.vec, q1.vec, i, shortest=True), norm=True)
            pi = p0 * (1 - i) + i * p1

            T_ti = SE3.Rt(qi.R, pi, check=True)
            if not SE3.isvalid(T_ti, check=True):
                if T_ti.A is None:
                    return None
                else:
                    q = UnitQuaternion(SO3(T_ti.R, check=False), norm=True).unit()
                    T_ti = SE3.Rt(q.R, T_ti.t, check=True)
            return T_ti

        return None

    def get_bounding_poses(self, t, round_decimals=4) -> [bool, SE3, float, SE3, float]:
        timestamp = round(t, round_decimals)
        [ti, T_ti] = self.hist_ctrl_pts.get_at_t(timestamp)
        if T_ti is None:
            [t1, T_t1] = self.hist_ctrl_pts.get_before_t(timestamp)
            [t2, T_t2] = self.hist_ctrl_pts.get_after_t(timestamp)
            if t1 and t2:
                return [True, T_t1, t1, T_t2, t2]
        else:
            # we found a pose, get another one after or before...
            [t2, T_t2] = self.hist_ctrl_pts.get_after_t(ti)
            if t2:
                return [True, T_ti, ti, T_t2, t2]
            else:
                [t1, T_t1] = self.hist_ctrl_pts.get_before_t(ti)
                if t1:
                    return [True, T_t1, t1, T_ti, ti]

        return [False, None, None, None, None]

    def get_control_poses(self, t, round_decimals=4) ->  [bool, SE3, float, SE3, float, SE3, float, SE3, float]:
        [success, T_t2, t2, T_t3, t3] = self.get_bounding_poses(t, round_decimals)
        if success:
            [t1, T_t1] = self.hist_ctrl_pts.get_before_t(t2)
            [t4, T_t4] = self.hist_ctrl_pts.get_after_t(t3)
            if t1 and t4:
                return [True, T_t1, t1, T_t2, t2, T_t3, t3, T_t4, t4]

        return [False, None, None, None, None, None, None, None, None]

    def get_trajectory(self, t_arr, type=InterpolationType.CubicBspline_SE3, round_decimals=4):
        t_rows, t_cols = t_arr.shape
        p_vec = np.zeros((t_rows, 3))
        q_vec = np.hstack((np.zeros((t_rows, 3)), np.ones((t_rows, 1)) ))
        t_vec = t_arr
        idx = 0
        for i in range(t_rows):
            t_i = round(t_arr[i, 0], round_decimals)
            T_i = self.get_pose(t_i, type=type, round_decimals=round_decimals)
            if T_i:
                [p,q] = SpatialConverter.SE3_to_p_q_HTMQ(T_i)
                p_vec[idx, :] = p
                q_vec[idx, :] = q
                t_vec[idx] = t_i
                idx += 1

        indices = range(0, idx)
        q_vec = q_vec[indices, :]
        p_vec = p_vec[indices, :]
        t_vec = t_vec[indices, :]

        return Trajectory(p_vec=p_vec, q_vec=q_vec, t_vec=t_vec)

    def feed_trajectory(self, traj, uniform_timestamps=True, min_dt=0.01, round_decimals=4):
        assert isinstance(traj, Trajectory)
        self.feed_pose_vec(traj.p_vec, traj.q_vec, traj.t_vec, uniform_timestamps, min_dt, round_decimals)

    def feed_pose_vec(self, p_vec, q_vec, t_vec, uniform_timestamps=True, min_dt=0.01, round_decimals=4):
        t_rows, t_cols = t_vec.shape
        p_rows, p_cols = p_vec.shape
        q_rows, q_cols = q_vec.shape
        assert (t_rows == p_rows)
        assert (t_rows == q_rows)
        assert (t_cols == 1)
        assert (p_cols == 3)
        assert (q_cols == 4)

        dict_poses = dict()
        for i in range(t_rows):
            T_i = SpatialConverter.p_q_HTMQ_to_SE3(p_vec[i, :], q_vec[i, :])
            t_i = round(t_vec[i, 0], round_decimals)
            dict_poses[t_i] = T_i

        hist_pose = HistoryBuffer(dict_t=dict_poses)
        self.feed_pose_history(hist_pose=hist_pose,
                               uniform_timestamps=uniform_timestamps,
                               min_dt=min_dt,
                               round_decimals=round_decimals)

    def feed_pose_history(self, hist_pose, uniform_timestamps=True, min_dt=0.01, round_decimals=4):
        # hist_pose is the intermediate format for loading
        assert isinstance(hist_pose, HistoryBuffer)
        # since we assume a temporal uniform spaced timestamps, we do not support non-uniform b-spline interpolation.
        # Thus we need to place to control points at equidistant time stamps,
        # like OpenVINS did in https://github.com/rpng/open_vins/blob/master/ov_core/src/sim/BsplineSE3.cpp!
        # A generic basis matrix can be computed from Sec 3.1 in
        # https://xiaoxingchen.github.io/2020/03/02/bspline_in_so3/general_matrix_representation_for_bsplines.pdf!
        if uniform_timestamps:

            t_arr = np.array(hist_pose.t_vec)
            dt_avg = max(min_dt, round(np.mean(np.diff(t_arr)), round_decimals))

            D = t_arr[-1] - t_arr[0]
            num_steps = math.floor(D / dt_avg)
            t_stop = t_arr[0] + num_steps * dt_avg

            t_arr = np.linspace(t_arr[0], t_stop, num=num_steps+1, endpoint=True)
            T_vec = list()
            t_vec = list()
            for t_i in t_arr:
                T_i = BsplineSE3.interpolate_SPLIT_pose(hist_pose, t_i)
                if T_i:
                    T_vec.append(T_i)
                    t_vec.append(t_i)

            self.hist_ctrl_pts.set(t_vec, T_vec, round_decimals=round_decimals)
        else:
            self.hist_ctrl_pts.set_dict(dict_t=hist_pose)

        pass





    @staticmethod
    def interpolate_SPLIT_pose(pose_hist, timestamp, round_decimals=4) -> SE3:
        timestamp = round(timestamp, round_decimals)
        [ti, T_GLOBAL_BODY_Ti] = pose_hist.get_at_t(timestamp)
        if T_GLOBAL_BODY_Ti is None:
            [t1, T_GLOBAL_BODY_T1] = pose_hist.get_before_t(timestamp)
            [t2, T_GLOBAL_BODY_T2] = pose_hist.get_after_t(timestamp)
            if t1 is None or t2 is None:
                # if verbose:
                #    print("* skip measurement from topic=" + topic + " at t=" + str(timestamp))
                return None

            assert (t1 <= t2), "Timestamp not ascending!"

            dt = t2 - t1
            dt_i = timestamp - t1
            i = abs(dt_i / dt)

            if dt < 10 ** -3:
                return T_GLOBAL_BODY_T1
            if i < 10 ** -2:
                return T_GLOBAL_BODY_T1
            elif i > 1.0 - 10 ** -2:
                return T_GLOBAL_BODY_T2
            # if dt > 1.0:
            #    print("WARNING: interpolate_pose(): dt="+str(dt) + " is huge!")

            # interpolate between poses:
            q0 = UnitQuaternion(T_GLOBAL_BODY_T1.R)
            q1 = UnitQuaternion(T_GLOBAL_BODY_T2.R)
            p0 = T_GLOBAL_BODY_T1.t
            p1 = T_GLOBAL_BODY_T2.t

            qi = UnitQuaternion(qslerp(q0.vec, q1.vec, i, shortest=True), norm=True)
            pi = p0 * (1 - i) + i * p1

            T_GLOBAL_BODY_Ti = SE3.Rt(qi.R, pi, check=True)
            if not SE3.isvalid(T_GLOBAL_BODY_Ti, check=True):
                if T_GLOBAL_BODY_Ti.A is None:
                    return None
                else:
                    q = UnitQuaternion(SO3(T_GLOBAL_BODY_Ti.R, check=False), norm=True).unit()
                    T_GLOBAL_BODY_Ti = SE3.Rt(q.R, T_GLOBAL_BODY_Ti.t, check=True)
        return T_GLOBAL_BODY_Ti
