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
# os
########################################################################################################################
import os
from sys import version_info
from datetime import datetime
import argparse
import time
import math
import yaml
import pandas as pandas
import numpy as np

import cnspy_numpy_utils.numpy_statistics
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_timestamp_association.TimestampAssociation import TimestampAssociation
from cnspy_relative_pose_evaluation.AssociateRelPoses import AssociateRelPoses, AssociateRelPoseCfg
from matplotlib import pyplot as plt

from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig


class RelPoseMeasEvaluation:
    report = None

    def __init__(self,
                 fn_gt,
                 fn_est,
                 ID1_arr=[0], ID2_arr=[1],
                 result_dir=None,
                 prefix=None,
                 save_plot=True,
                 save_statistics=True,
                 show_plot=True,
                 cfg=AssociateRelPoseCfg(),
                 plot_timestamps=True,
                 plot_ranges=True,
                 plot_angles=True,
                 plot_ranges_sorted=True,
                 plot_range_error=True,
                 plot_angle_error=True,
                 plot_range_histogram=True,
                 plot_angle_histogram=True,
                 plot_pose_err=True,
                 plot_position_err=True,
                 plot_pose=True,
                 plot_NEES=True,
                 verbose=False,
                 filter_histogram=True,
                 ):
        if not result_dir:
            result_dir = '.'
        if not prefix:
            prefix = ''

        statistics_file = None
        if save_statistics:
            if not os.path.exists(result_dir):
                os.makedirs(result_dir)
            stat_fn = os.path.join(result_dir, 'statistics.yaml')
            statistics_file = open(stat_fn, 'w')
            if verbose:
                print("RelPoseMeasEvaluation: stat_fn=" + stat_fn)
            yaml.dump({'info': 'RelPoseMeasEvaluation Statistics',
                       'time': datetime.now().strftime("%m/%d/%Y, %H:%M:%S"),
                       'fn_gt': fn_gt,
                       'fn_est': fn_est,
                       'filter histogram': filter_histogram,
                       'remove outliers': cfg.remove_outliers,
                       'outliers max range': cfg.max_range,
                       'pose error type': str(cfg.pose_error_type),
                       'outliers min range': 0,
                       'outliers max angle': cfg.max_angle,
                       'result_dir': result_dir}, statistics_file, sort_keys=False, explicit_start=False,
                      default_flow_style=False)
            yaml.dump({'ID1s': ID1_arr,
                       'ID2s': ID2_arr}, statistics_file, sort_keys=False, explicit_start=False,
                      default_flow_style=True)
        fn_gt = os.path.abspath(fn_gt)
        fn_est = os.path.abspath(fn_est)

        plt.style.use('ggplot')

        SMALL_SIZE = 6
        MEDIUM_SIZE = 7
        BIGGER_SIZE = 8

        plt.rc('font', size=SMALL_SIZE)  # controls default text sizes
        plt.rc('axes', titlesize=SMALL_SIZE)  # fontsize of the axes title
        plt.rc('axes', labelsize=MEDIUM_SIZE)  # fontsize of the x and y labels
        plt.rc('xtick', labelsize=SMALL_SIZE)  # fontsize of the tick labels
        plt.rc('ytick', labelsize=SMALL_SIZE)  # fontsize of the tick labels
        plt.rc('legend', fontsize=SMALL_SIZE)  # legend fontsize
        plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

        for ID1 in ID1_arr:
            if plot_timestamps:
                fig_t = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_t.suptitle('Timestamps of ID=' + str(ID1), fontsize=16)
            if plot_ranges:
                fig_r = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_r.suptitle('Ranges of ID=' + str(ID1), fontsize=16)
            if plot_angles:
                fig_a = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_a.suptitle('Angles of ID=' + str(ID1), fontsize=16)
            if plot_ranges_sorted:
                fig_rs = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_rs.suptitle('Range sorted of ID=' + str(ID1), fontsize=16)
            if plot_range_error:
                fig_re = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_re.suptitle('Range Error of ID=' + str(ID1), fontsize=16)
            if plot_angle_error:
                fig_ae = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_ae.suptitle('Angle Error of ID=' + str(ID1), fontsize=16)
            if plot_position_err:
                fig_pos_err_dict = dict()
                for ID2 in ID2_arr:
                    if ID1 == ID2:
                        continue
                    fig_pe = plt.figure(figsize=(20, 15), dpi=int(200))
                    fig_pe.suptitle('Position Error of ID=' + str(ID1) + " to " + str(ID2), fontsize=16)
                    fig_pos_err_dict[ID2] = fig_pe
            if plot_pose_err:
                fig_pe_dict = dict()
                for ID2 in ID2_arr:
                    if ID1 == ID2:
                        continue
                    fig_pe = plt.figure(figsize=(20, 15), dpi=int(200))
                    fig_pe.suptitle('Pose Error of ID=' + str(ID1) + " to " + str(ID2), fontsize=16)
                    fig_pe_dict[ID2] = fig_pe
            if plot_pose:
                fig_p_dict = dict()
                for ID2 in ID2_arr:
                    if ID1 == ID2:
                        continue
                    fig_p = plt.figure(figsize=(20, 15), dpi=int(200))
                    fig_p.suptitle('Pose of ID=' + str(ID1) + " to " + str(ID2), fontsize=16)
                    fig_p_dict[ID2] = fig_p
            if plot_NEES:
                fig_n = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_n.suptitle('NEES of ID=' + str(ID1), fontsize=16)
            if plot_range_histogram:
                fig_hr = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_hr.suptitle('Range Error Histograms of ID=' + str(ID1), fontsize=16)
            if plot_angle_histogram:
                fig_ha = plt.figure(figsize=(20, 15), dpi=int(200))
                fig_ha.suptitle('Angle Error Histograms of ID=' + str(ID1), fontsize=16)
            if save_statistics:
                dict_statistics_i = {'ID': ID1,
                                     'range_constant_bias_table': dict(),
                                     'range_noise_table': dict(),
                                     'angle_constant_bias_table': dict(),
                                     'angle_noise_table': dict(),
                                     'percent_outliers' : dict(),
                                     'ARMSE_p': dict(),
                                     'ARMSE_q_deg': dict(),
                                     'scale': dict(),
                                     'samples': dict(),
                                     }
                pass


            n = len(ID2_arr)
            if ID1 in ID2_arr:
                n = n - 1

            sqrt_n = max(1, math.ceil(math.sqrt(n)))
            n_rows = sqrt_n
            if sqrt_n * sqrt_n < n:
                n_cols = sqrt_n + 1
            else:
                n_cols = sqrt_n

            if n_rows * n_cols <= n:
                assert(False, "something went wrong!")

            idx = 1
            for ID2 in ID2_arr:
                if ID1 == ID2:
                    continue

                cfg.ID1 = int(ID1)
                cfg.ID2 = int(ID2)
                cfg_title = str("ID" + str(ID1) + " to ID" + str(ID2))
                assoc = AssociateRelPoses(fn_gt=fn_gt, fn_est=fn_est, cfg=cfg)
                if not assoc.data_loaded:
                    print("WARING: no measurement for " + cfg_title + "found!")
                    continue
                assoc.save(result_dir=result_dir, prefix=prefix + cfg_title)

                if save_statistics:
                    dict_statistics_i['percent_outliers'][ID2] = round(float(assoc.perc_outliers), 2)
                    dict_nees = assoc.avg_NEES()
                    for key, val in dict_nees.items():
                        if not (key in dict_statistics_i):
                            dict_statistics_i[key] = dict()
                        dict_statistics_i[key][ID2] = round(float(val),2)
                    ARMSE_p, ARMSE_q_deg = assoc.traj_err.get_ARMSE()
                    dict_statistics_i['ARMSE_p'][ID2] = round(float(ARMSE_p),2)
                    dict_statistics_i['ARMSE_q_deg'][ID2] = round(float(ARMSE_q_deg),2)
                    dict_statistics_i['scale'][ID2] = round(float(assoc.traj_err.scale),2)
                    dict_statistics_i['samples'][ID2] = assoc.num_samples()
                if plot_timestamps:
                    ax_t = fig_t.add_subplot(n_rows, n_cols, idx)
                    assoc.plot_timestamps(fig=fig_t, ax=ax_t, calc_error=True, cfg_title=cfg_title)

                if plot_ranges:
                    ax_r = fig_r.add_subplot(n_rows, n_cols, idx)
                    assoc.plot_ranges(fig=fig_r, ax=ax_r, cfg_title=cfg_title)

                if plot_angles:
                    ax_a = fig_a.add_subplot(n_rows, n_cols, idx)
                    assoc.plot_angles(fig=fig_a, ax=ax_a, cfg_title=cfg_title)
                if plot_angle_error:
                    ax_ae = fig_ae.add_subplot(n_rows, n_cols, idx)
                    assoc.plot_angle_error(fig=fig_ae, ax=ax_ae, cfg_title=cfg_title)

                if plot_ranges_sorted:
                    ax_rs = fig_rs.add_subplot(n_rows, n_cols, idx)
                    assoc.plot_ranges(fig=fig_rs, ax=ax_rs, sorted=True, cfg_title=cfg_title)

                if plot_position_err:
                    fig_pe = fig_pos_err_dict[ID2]
                    cfg_plt = TrajectoryPlotConfig()
                    cfg_plt.title = cfg_title
                    cfg_plt.show = False
                    cfg_plt.close_figure = False
                    cfg_plt.unwrap = False
                    assoc.plot_position_err(fig=fig_pe, cfg=cfg_plt)

                if plot_pose_err:
                    fig_pe = fig_pe_dict[ID2]
                    cfg_plt = TrajectoryPlotConfig()
                    cfg_plt.title = cfg_title
                    cfg_plt.show = False
                    cfg_plt.close_figure = False
                    cfg_plt.unwrap = False
                    assoc.plot_traj_err(fig=fig_pe, cfg=cfg_plt)

                if plot_pose:
                    fig_pe = fig_p_dict[ID2]
                    cfg_plt = TrajectoryPlotConfig()
                    cfg_plt.title = cfg_title
                    cfg_plt.show = False
                    cfg_plt.close_figure = False
                    cfg_plt.unwrap = True
                    assoc.plot_traj(fig=fig_pe, cfg=cfg_plt)

                if plot_NEES:
                    ax_n = fig_n.add_subplot(n_rows, n_cols, idx)
                    assoc.plot_NEES(fig=fig_n, ax=ax_n, relative_time=True, cfg_title=cfg_title)


                if plot_range_error:
                    ax_e = fig_re.add_subplot(n_rows, n_cols, idx)
                    [fig_, ax_, stat, r_vec_err_] = assoc.plot_range_error(fig=fig_re, ax=ax_e,
                                                                           sorted=False,
                                                                           remove_outlier=True,
                                                                           cfg_title=cfg_title)
                    cnspy_numpy_utils.numpy_statistics.print_statistics(stat, desc=cfg_title + " error")
                if plot_range_histogram:
                    ax_hr = fig_hr.add_subplot(n_rows, n_cols, idx)
                    [fig_, ax_, stat, r_vec_err_] = assoc.plot_range_error_histogram(fig=fig_hr,
                                                                                     ax=ax_hr,
                                                                                     max_error=cfg.max_range_error,
                                                                                     filter_histogramm=filter_histogram,
                                                                                     ID1=ID1, ID2=ID2)
                    if stat is not None and save_statistics:
                        dict_statistics_i['range_constant_bias_table'][ID2] = round(float(stat['mean']), 2)
                        dict_statistics_i['range_noise_table'][ID2] = round(float(stat['std']), 2)
                if plot_angle_histogram:
                    ax_ha = fig_ha.add_subplot(n_rows, n_cols, idx)
                    [fig_, ax_, stat, r_vec_err_] = assoc.plot_angle_error_histogram(fig=fig_ha,
                                                                                     ax=ax_ha,
                                                                                     max_error=10,
                                                                                     filter_histogramm=filter_histogram,
                                                                                     ID1=ID1, ID2=ID2)
                    if stat is not None  and save_statistics:
                        dict_statistics_i['angle_constant_bias_table'][ID2] = round(float(stat['mean']), 2)
                        dict_statistics_i['angle_noise_table'][ID2] = round(float(stat['std']), 2)
                # the histogram of the date
                idx += 1

            # for ID2
            if verbose:
                print("* RangeEvaluation(): ranges associated!")

            # Tweak spacing to prevent clipping of ylabel
            if plot_timestamps:
                fig_t.tight_layout()
                if save_plot:
                    AssociateRelPoses.show_save_figure(fig=fig_t, result_dir=result_dir,
                                                       save_fn=str("Timestamps" + str(ID1)),
                                                       show=show_plot, close_figure=not show_plot)

            if plot_ranges:
                fig_r.tight_layout()
                if save_plot:
                    AssociateRelPoses.show_save_figure(fig=fig_r, result_dir=result_dir,
                                                       save_fn=str("Ranges_ID" + str(ID1)),
                                                       show=show_plot, close_figure=not show_plot)
            if plot_angles:
                fig_a.tight_layout()
                if save_plot:
                    AssociateRelPoses.show_save_figure(fig=fig_a, result_dir=result_dir,
                                                       save_fn=str("Angle_ID" + str(ID1)),
                                                       show=show_plot, close_figure=not show_plot)
            if plot_ranges_sorted:
                fig_rs.tight_layout()
                if save_plot:
                    AssociateRelPoses.show_save_figure(fig=fig_rs, result_dir=result_dir,
                                                       save_fn=str("Range_Sorted_ID" + str(ID1)),
                                                       show=show_plot, close_figure=not show_plot)

            if plot_range_error:
                fig_re.tight_layout()
                if save_plot:
                    AssociateRelPoses.show_save_figure(fig=fig_re, result_dir=result_dir,
                                                       save_fn=str("Range_Errors_ID" + str(ID1)),
                                                       show=show_plot, close_figure=not show_plot)
            if plot_position_err:
                for ID2 in ID2_arr:
                    if ID1 == ID2:
                        continue
                    fig_pe = fig_pos_err_dict[ID2]
                    fig_pe.tight_layout()
                    if save_plot:
                        AssociateRelPoses.show_save_figure(fig=fig_pe, result_dir=result_dir,
                                                           save_fn=str("Position_Errors_ID" + str(ID1) + "_to_" + str(ID2)),
                                                           show=show_plot, close_figure=not show_plot)
                    pass
            if plot_pose_err:
                for ID2 in ID2_arr:
                    if ID1 == ID2:
                        continue
                    fig_pe = fig_pe_dict[ID2]
                    fig_pe.tight_layout()
                    if save_plot:
                        AssociateRelPoses.show_save_figure(fig=fig_pe, result_dir=result_dir,
                                                           save_fn=str("Pose_Errors_ID" + str(ID1) + "_to_" + str(ID2)),
                                                           show=show_plot, close_figure=not show_plot)
                    pass

            if plot_pose:
                for ID2 in ID2_arr:
                    if ID1 == ID2:
                        continue
                    fig_pe = fig_p_dict[ID2]
                    fig_pe.tight_layout()
                    if save_plot:
                        AssociateRelPoses.show_save_figure(fig=fig_pe, result_dir=result_dir,
                                                           save_fn=str("Pose_ID" + str(ID1) + "_to_" + str(ID2)),
                                                           show=show_plot, close_figure=not show_plot)
                    pass
            if plot_angle_error:
                fig_ae.tight_layout()
                if save_plot:
                    AssociateRelPoses.show_save_figure(fig=fig_ae, result_dir=result_dir,
                                                       save_fn=str("Angle_Errors_ID" + str(ID1)),
                                                       show=show_plot, close_figure=not show_plot)
            if plot_range_histogram:
                fig_hr.tight_layout()
                if save_plot:
                    AssociateRelPoses.show_save_figure(fig=fig_hr, result_dir=result_dir,
                                                       save_fn=str("Range_Error_Histograms_ID" + str(ID1)),
                                                       show=show_plot, close_figure=not show_plot)
            if plot_angle_histogram:
                fig_ha.tight_layout()
                if save_plot:
                    AssociateRelPoses.show_save_figure(fig=fig_ha, result_dir=result_dir,
                                                       save_fn=str("Angle_Error_Histograms_ID" + str(ID1)),
                                                       show=show_plot, close_figure=not show_plot)
            if plot_NEES:
                fig_n.tight_layout()
                if save_plot:
                    AssociateRelPoses.show_save_figure(fig=fig_n, result_dir=result_dir,
                                                       save_fn=str("NEES_ID" + str(ID1)),
                                                       show=show_plot, close_figure=not show_plot)
            if save_statistics:
                yaml.dump(dict_statistics_i, statistics_file, explicit_start=True, default_flow_style=True)
                if verbose:
                    print(yaml.dump(dict_statistics_i, explicit_start=True, default_flow_style=True))
            pass  # UWB_ID2
        pass  # UWB_ID1

        if save_statistics:
            statistics_file.close()

        pass  # DONE


def main():
    parser = argparse.ArgumentParser(
        description='RelPoseMeasEvaluation: evaluate and estimated and true pairwise relative pose measurements')
    parser.add_argument('--fn_gt', help='input ground-truth trajectory CSV file', default="not specified")
    parser.add_argument('--fn_est', help='input estimated  trajectory CSV file', default="not specified")
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--ID1s', help='ID of TX', nargs='+', default=[0])
    parser.add_argument('--ID2s', help='ID of RX', nargs='+', default=[1])
    parser.add_argument('--prefix', help='prefix in results', default='')
    parser.add_argument('--max_timestamp_difference',
                        help='Max difference between associated timestampes (t_gt - t_est)', default=0.03)
    parser.add_argument('--subsample', help='subsampling factor for input data (CSV)', default=0)
    parser.add_argument('--plot', action='store_true', default=True)
    parser.add_argument('--save_plot', action='store_true', default=True)
    parser.add_argument('--save_statistics', action='store_true', default=True)
    parser.add_argument('--show_plot', action='store_true', default=True)
    parser.add_argument('--relative_timestamps', action='store_true', default=False)
    parser.add_argument('--remove_outliers', action='store_true', default=False)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--max_range', help='range that classifies as outlier', default='30')
    parser.add_argument('--max_angle', help='angle that classifies as outlier', default='6.4')
    parser.add_argument('--range_error_val', help='value assigned to outlier', default='0')
    parser.add_argument('--angle_error_val', help='value assigned to outlier', default='0')
    parser.add_argument('--label_timestamp', help='timestamp label in CSV', default='t')
    parser.add_argument('--label_range', help='range label in CSV', default='range')
    parser.add_argument('--label_angle', help='range label in CSV', default='angle')
    parser.add_argument('--label_ID1', help='ID1 label in CSV', default='ID1')
    parser.add_argument('--label_ID2', help='ID2 label in CSV', default='ID2')
    parser.add_argument('--plot_timestamps', action='store_true', default=False)
    parser.add_argument('--plot_ranges', action='store_true', default=False)
    parser.add_argument('--plot_angles', action='store_true', default=False)
    parser.add_argument('--plot_err_poses', action='store_true', default=False)
    parser.add_argument('--plot_ranges_sorted', action='store_true', default=False)
    parser.add_argument('--plot_errors', action='store_true', default=False)
    parser.add_argument('--plot_histograms', action='store_true', default=False)
    parser.add_argument('--filter_histogram', action='store_true', default=False)
    tp_start = time.time()
    args = parser.parse_args()
    cfg = AssociateRelPoseCfg(ID1=None,
                              ID2=None,
                              relative_timestamps=args.relative_timestamps,
                              max_difference=float(args.max_timestamp_difference),
                              subsample=int(args.subsample),
                              verbose=args.verbose,
                              remove_outliers=args.remove_outliers,
                              max_range=float(args.max_range),
                              range_error_val=float(args.range_error_val),
                              label_timestamp=args.label_timestamp,
                              label_ID1=args.label_ID1,
                              label_ID2=args.label_ID2,
                              label_range=args.label_range)

    eval = RelPoseMeasEvaluation(fn_gt=args.fn_gt,
                                 fn_est=args.fn_est,
                                 ID1_arr=args.ID1s,
                                 ID2_arr=args.ID2s,
                                 cfg=cfg,
                                 result_dir=args.result_dir,
                                 prefix=args.prefix,
                                 save_plot=args.save_plot,
                                 show_plot=args.show_plot,
                                 save_statistics=args.save_statistics,
                                 plot_timestamps=args.plot_timestamps,
                                 plot_ranges=args.plot_ranges,
                                 plot_angles=args.plot_angles,
                                 plot_pose_err=args.plot_err_poses,
                                 plot_ranges_sorted=args.plot_ranges_sorted,
                                 plot_range_error=args.plot_errors,
                                 plot_angle_error=args.plot_errors,
                                 plot_range_histogram=args.plot_histograms,
                                 plot_angle_histogram=args.plot_histograms,
                                 filter_histogram=args.filter_histogram
                                 )

    print(" ")
    print("finished after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
