#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2023, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
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
# BASED ON: https://github.com/aau-cns/cnspy_rosbag2csv
# just install "pip install cnspy-rosbag2csv"
########################################################################################################################
from datetime import datetime
import math
import sys

import rosbag
import time
import os
import argparse
import yaml
import csv
from tqdm import tqdm
import numpy as np
from spatialmath import UnitQuaternion, SO3, SE3
from spatialmath.base.quaternions import qslerp

from cnspy_relative_pose_evaluation.RelPose_ROSBag2CSV import RelPose_ROSBag2CSV
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_trajectory.BsplineSE3 import BsplineSE3, TrajectoryInterpolationType
from cnspy_trajectory.HistoryBuffer import get_key_from_value
from cnspy_trajectory.ROSBag_Pose import ROSBag_Pose
from cnspy_trajectory.SpatialConverter import SpatialConverter
from cnspy_relative_pose_evaluation.ROSBag_TrueRelPoses import ROSBag_TrueRelPoses

from mrs_msgs.msg import PoseWithCovarianceArrayStamped, PoseWithCovarianceIdentified


class ROSBag_MeasObjPoses:
    def __init__(self):
        pass

    @staticmethod
    def extract(bagfile_in_name,
                bagfile_out_name=None,
                csv_out_name=None,
                yaml_out_name=None,
                cfg=None,
                use_header_timestamp=False,
                verbose=False,
                interp_type=TrajectoryInterpolationType.cubic,
                min_dt=0.05,
                ):
        ignore_new_topic_name = False
        if not os.path.isfile(bagfile_in_name):
            print("ROSBag_MeasObjPoses: could not find file: %s" % bagfile_in_name)
            return False
        cfg = os.path.abspath(cfg)
        if not os.path.isfile(cfg):
            print("ROSBag_MeasObjPoses: could not find file: %s" % cfg)
            return False

        if verbose:
            print("ROSBag_MeasObjPoses:")
            print("* bagfile in name: " + str(bagfile_in_name))
            print("* bagfile out name: " + str(bagfile_out_name))
            print("* csv out name: " + str(csv_out_name))
            print("* yaml out name: " + str(csv_out_name))
            print("* cfg YAML file: \t " + str(cfg))
            print("* use_header_timestamp: " + str(use_header_timestamp))

        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_in_name)
        except:
            if verbose:
                print("ROSBag_MeasObjPoses Unexpected error!")

            return False

        ## create result dir:
        outfile_fn = None
        if bagfile_out_name:
            outfile_fn = bagfile_out_name
        elif csv_out_name:
            outfile_fn = csv_out_name
        elif yaml_out_name:
            outfile_fn = yaml_out_name
        else:
            print("ROSBag_MeasObjPoses: no output file specified! Using directory of as result_dir: %s" % cfg)
            outfile_fn = cfg

        [root, ext] = os.path.splitext(outfile_fn)
        [result_dir, tail] = os.path.split(root)
        try:  # else already exists
            os.makedirs(result_dir)
        except:
            pass

        if verbose:
            print("* result_dir: \t " + str(result_dir))

        ## CSV file:
        if not csv_out_name:
            csv_out_fn = str(result_dir + '/') + 'object_poses.csv'
        else:
            csv_out_fn = csv_out_name

        if verbose:
            print("* csv_out_fn: " + str(csv_out_fn))

        csvfile = open(csv_out_fn, 'w+')
        file_writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
        file_writer.writerow(RelPose_ROSBag2CSV.get_header(False))

        ## YAML file:
        if not yaml_out_name:
            stat_fn = os.path.join(result_dir, 'object_poses.yaml')
        else:
            stat_fn = yaml_out_name
        statistics_file = open(stat_fn, 'w')
        if verbose:
            print("ROSBag_MeasObjPoses: stat_fn=" + stat_fn)
        yaml.dump({'info': 'ROSBag_MeasObjPoses Statistics',
                   'time': datetime.now().strftime("%m/%d/%Y, %H:%M:%S"),
                   'bag_in_fn': bagfile_in_name,
                   'interp_type': str(interp_type),
                   'use_header_timestamp': str(use_header_timestamp),
                   'result_dir': result_dir}, statistics_file, sort_keys=False, explicit_start=False,
                  default_flow_style=False)
        dist_avg_obj_pose = dict()  # dict<id, SE3>

        dict_cfg = ROSBag_TrueRelPoses.load_dict_cfg(cfg_fn=cfg,
                                                     ignore_new_topic_name=ignore_new_topic_name,
                                                     verbose=verbose)
        if dict_cfg is None:
            print(" ERROR configuration is wrong!")
            return False

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("ROSBag_MeasObjPoses Unexpected error, bag file might be empty!")
            bag.close()
            return False

        ## check if desired topics are in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']

        found = ROSBag_TrueRelPoses.check_topics(bag_topics, dict_cfg, num_messages, verbose)
        if verbose and not found:
            print("ROSBag_MeasObjPoses desired topics not found!")

        dict_static_objects = ROSBag_TrueRelPoses.load_dict_static_objects(dict_cfg)
        round_decimals = 4
        dict_bsplines, dict_history = ROSBag_TrueRelPoses.load_dict_splines(bag, dict_cfg, interp_type, min_dt,
                                                                            num_messages, round_decimals)

        if len(dict_bsplines) == 0:
            if verbose:
                print("ROSBag_MeasObjPoses No poses found!")
            bag.close()
            return False

        cnt = 0
        try:  # else already exists
            print("ROSBag_MeasObjPoses computing object poses from relative measurements measurements...")
            if bagfile_out_name:
                outbag =  rosbag.Bag(bagfile_out_name, 'w')
            else:
                outbag = None

            for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                if topic in dict_cfg["relpose_topics"].values() and hasattr(msg, 'poses') and hasattr(msg,
                                                                                                      'header'):
                    ID1 = get_key_from_value(dict_cfg["relpose_topics"], topic)
                    idx_pose = 0
                    timestamp = msg.header.stamp.to_sec()

                    msg_obj_poses = PoseWithCovarianceArrayStamped()
                    msg_obj_poses.header = msg.header

                    relpose: PoseWithCovarianceIdentified
                    for relpose in msg.poses:  # id, pose, covariance

                        ID2 = relpose.id

                        is_sensor = (ID2 in dict_cfg["true_pose_topics"].keys())
                        is_object = (ID2 in dict_cfg["object_positions"].keys())
                        if is_object or not is_sensor:
                            # ID2 = get_key_from_value(dict_cfg["true_pose_topics"], topic2)

                            T_GLOBAL_SENSOR1 = dict_bsplines[dict_cfg["true_pose_topics"][ID1]].get_pose(
                                t=timestamp,
                                interp_type=interp_type,
                                round_decimals=round_decimals)

                            if T_GLOBAL_SENSOR1 is not None:
                                T_SENSOR1_OBJECT: SE3 = ROSBag_Pose.geometry_msgs_pose_to_SE3(relpose.pose)
                                T_GLOBAL_OBJECT: SE3 = (T_GLOBAL_SENSOR1 * T_SENSOR1_OBJECT)

                                if ID2 not in dist_avg_obj_pose.keys():
                                    dist_avg_obj_pose[ID2]: np.ndarray = T_GLOBAL_OBJECT.twist().S
                                else:
                                    avg_twist: np.ndarray = (dist_avg_obj_pose[ID2] + T_GLOBAL_OBJECT.twist().S)/2
                                    dist_avg_obj_pose[ID2] = avg_twist

                                p = T_GLOBAL_OBJECT.t
                                q = UnitQuaternion(T_GLOBAL_OBJECT.R, norm=True).unit()
                                qv = q.vec

                                pose_id = PoseWithCovarianceIdentified()
                                pose_id.id = ID2
                                pose_id.pose.position.x = p[0]
                                pose_id.pose.position.y = p[1]
                                pose_id.pose.position.z = p[2]
                                pose_id.pose.orientation.w = qv[0]
                                pose_id.pose.orientation.x = qv[1]
                                pose_id.pose.orientation.y = qv[2]
                                pose_id.pose.orientation.z = qv[3]
                                pose_id.covariance = relpose.covariance

                                msg_obj_poses.poses.append(pose_id)
                                content = RelPose_ROSBag2CSV.to_csv_line(timestamp, ID2, pose_id, 4,
                                                                         False)
                                file_writer.writerow(content)

                                # for id2
                                if outbag and "meas_object_pose_topics" in dict_cfg:
                                    id_topic = ID2
                                    if id_topic in dict_cfg["meas_object_pose_topics"]:
                                        # assign new topic name
                                        topic_out = dict_cfg["meas_object_pose_topics"][id_topic]
                                    else:
                                        print("No meas_object_pose_topics with ID=[" + str(id_topic) + "] found!")
                                        continue

                                    if use_header_timestamp and hasattr(msg, "header"):
                                        outbag.write(topic_out, pose_id, msg.header.stamp)
                                    else:
                                        outbag.write(topic_out, pose_id, t)
                                    cnt += 1

                        idx_pose += 1


        except  Exception as e:
            print("ROSBag_MeasObjPoses Unexpected error while creating the bag file! msg=%s" % repr(e))
            print(str(sys.exc_info()))
            return False
        ## CLEANUP:
        except AssertionError as error:
            print(error)
            print("ROSBag_MeasObjPoses Unexpected error while creating bag file")
            return False

        if verbose:
            print("\nROSBag_MeasObjPoses " + str(cnt) + " range measurements used!")



        bag.close()
        if outbag:
            outbag.close()
        if csvfile:
            csvfile.close()
        if statistics_file:
            dict_obj_pos = dict()
            dict_obj_or = dict()
            for id, twist in dist_avg_obj_pose.items():
                T_GLOBAL_OBJECT_avg: SE3 = SE3.Exp(twist)
                obj_pos_str = np.array2string(T_GLOBAL_OBJECT_avg.t.T, separator=', ')
                obj_or_str = str(list(UnitQuaternion(T_GLOBAL_OBJECT_avg.R).vec.T))
                dict_obj_pos[id] = obj_pos_str
                dict_obj_or[id] = obj_or_str
                dict_statistics_i = {'ID': str(id),
                                     'object_position_meas': obj_pos_str,
                                     'object_orientation_meas': obj_or_str }
                yaml.dump(dict_statistics_i, statistics_file, explicit_start=True, default_flow_style=True)
                if verbose:
                    print(yaml.dump(dict_statistics_i, explicit_start=True, default_flow_style=True))

            yaml.dump({'object_positions_meas': dict_obj_pos, 'object_orientations_meas': dict_obj_or}, statistics_file, explicit_start=True, default_flow_style=True)
            statistics_file.close()
        return True


def main():
    # example: ROSBag_MeasObjPoses.py --bagfile ../test/sample_data//uwb_calib_a01_2023-08-31-21-05-46.bag --topic /d01/mavros/vision_pose/pose --cfg ../test/sample_data/config.yaml --verbose
    parser = argparse.ArgumentParser(
        description='ROSBag_MeasObjPoses: extract given pose topics and compute for each relative poses measurement a' +
                    'perturbed object pose, which is stored ' +
                    'under a new topic in the specified bag file. These poses are dumped into CSV file and the average object pose is ')
    parser.add_argument('--bagfile_in', help='input bag file', required=True)
    parser.add_argument('--bagfile_out', help='output bag file', required=False, default=None)
    parser.add_argument('--csv_out', help='output csv file', required=False, default=None)
    parser.add_argument('--yaml_out', help='output yaml file', required=False, default=None)
    parser.add_argument('--cfg',
                        help='YAML configuration file describing the setup: ' +
                             '{sensor_positions:{<id>:[x,y,z], ...}, sensor_orientations:{<id>:[w,x,y,z], ...}, ' +
                             'relpose_topics:{<id>:<topic_name>, ...}, true_pose_topics:{<id>:<topic_name>, ...},' +
                             'meas_object_pose_topics:{<id>:<topic_name>, ...}}',
                        default="config.yaml", required=True)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--use_header_timestamp', action='store_true',
                        help='overwrites the bag time with the header time stamp', default=False)
    parser.add_argument('--interpolation_type', help='Trajectory interpolation type',
                        choices=TrajectoryInterpolationType.list(),
                        default=str(TrajectoryInterpolationType.linear))
    parser.add_argument('--min_dt',
                        help='temporal displacement of cubic spline control points',
                        default=0.05)
    tp_start = time.time()
    args = parser.parse_args()

    if ROSBag_MeasObjPoses.extract(bagfile_in_name=args.bagfile_in,
                                   bagfile_out_name=args.bagfile_out,
                                   csv_out_name=args.csv_out,
                                   yaml_out_name=args.yaml_out,
                                   cfg=args.cfg,
                                   verbose=args.verbose,
                                   use_header_timestamp=args.use_header_timestamp,
                                   interp_type=TrajectoryInterpolationType(args.interpolation_type),
                                   min_dt=float(args.min_dt)):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
