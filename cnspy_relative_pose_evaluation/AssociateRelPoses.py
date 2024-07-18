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
from sys import version_info
import time
import numpy as np
from matplotlib import pyplot as plt
import pandas as pandas
import math

from cnspy_numpy_utils.numpy_statistics import *
from cnspy_timestamp_association.TimestampAssociation import TimestampAssociation
from cnspy_trajectory.PlotLineStyle import PlotLineStyle
from cnspy_ranging_evaluation.AssociateRanges import AssociateRanges

class AssociateRelPoseCfg:
    ID1 = None
    ID2 = None
    relative_timestamps = False
    max_difference = 0.02
    subsample = 0
    verbose = False
    remove_outliers = False,
    max_range = 30
    range_error_val = 0
    label_timestamp = 't'
    label_ID1 = 'ID1'
    label_ID2 = 'ID2'
    label_range = 'range'

    def __init__(self, ID1=None, ID2=None, relative_timestamps=False,
                 max_difference=0.02, subsample=0, verbose=False, remove_outliers=False,
                 max_range=30, range_error_val=0, label_timestamp='t',
                 label_ID1='ID1',
                 label_ID2='ID2',
                 label_range = 'range'):
        self.label_timestamp = label_timestamp
        self.label_ID1 = label_ID1
        self.label_ID2 = label_ID2
        self.label_range = label_range
        self.ID1 = ID1
        self.ID2 = ID2
        self.relative_timestamps = relative_timestamps
        self.max_difference = max_difference
        self.subsample = subsample
        self.verbose = verbose
        self.remove_outliers = remove_outliers
        self.max_range = max_range
        self.range_error_val = range_error_val


class AssociateRelPoses(AssociateRanges):
    csv_df_gt = None
    csv_df_est = None

    data_frame_gt_matched = None
    data_frame_est_matched = None

    matches_est_gt = None  # list of tuples containing [(idx_est, idx_gt), ...]

    cfg = AssociateRelPoseCfg()
    data_loaded = False

    def __init__(self, fn_gt, fn_est, cfg):
        #AssociateRanges.__init__(self)
        cfg = cfg
        self.load(fn_gt, fn_est, cfg)

    def load(self, fn_gt, fn_est, cfg):

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
        range_est = np.linalg.norm(self.csv_df_est[['tx', 'ty', 'tz']].to_numpy(), axis=1)
        self.csv_df_est[cfg.label_range] = range_est
        range_gt = np.linalg.norm(self.csv_df_gt[['tx', 'ty', 'tz']].to_numpy(), axis=1)
        self.csv_df_gt[cfg.label_range] = range_gt

        if cfg.remove_outliers:
            self.csv_df_est[cfg.label_range] = self.csv_df_est[cfg.label_range].where(
                abs(self.csv_df_est[cfg.label_range]) < 0, other=cfg.range_error_val)
            self.csv_df_est[cfg.label_range] = self.csv_df_est[cfg.label_range].where(
                self.csv_df_est[cfg.label_range] > cfg.max_range, other=cfg.range_error_val)

        else:
            indices = ((self.csv_df_est[cfg.label_range]) < 0)
            self.csv_df_est.loc[indices, cfg.label_range] = cfg.range_error_val
            indices = (self.csv_df_est[cfg.label_range] > cfg.max_range)
            self.csv_df_est.loc[indices, cfg.label_range] = cfg.range_error_val

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
            print("AssociateRanges(): {} timestamps associated.".format(len(idx_est)))

        self.data_loaded = True
        # using zip() and * operator to
        # perform Unzipping
        # res = list(zip(*test_list))
