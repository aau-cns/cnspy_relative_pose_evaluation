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
import sys
import traceback
import rosbag
import time
import os
import argparse
import yaml
import csv
from tqdm import tqdm
import numpy as np
from numpy import linalg as LA
from spatialmath import UnitQuaternion, SO3, SE3, Quaternion, base, quaternion
from spatialmath.base.quaternions import qslerp
from cnspy_ranging_evaluation.ROSBag_TrueRanges import HistoryBuffer, get_key_from_value
from std_msgs.msg import Header, Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped

def interpolate_pose(pose_hist, timestamp):
    T_GLOBAL_BODY_Ti = pose_hist.get_at_t(timestamp)
    if T_GLOBAL_BODY_Ti is None:
        [t1, T_GLOBAL_BODY_T1] = pose_hist.get_before_t(timestamp)
        [t2, T_GLOBAL_BODY_T2] = pose_hist.get_after_t(timestamp)

        if t1 is None or t2 is None:
            # if verbose:
            #    print("* skip measurement from topic=" + topic + " at t=" + str(timestamp))
            return None
        dt = t2 - t1
        dt_i = timestamp - t1
        i = abs(dt_i / dt)

        # interpolate between poses:
        q0 = UnitQuaternion(T_GLOBAL_BODY_T1.R)
        q1 = UnitQuaternion(T_GLOBAL_BODY_T2.R)
        p0 = T_GLOBAL_BODY_T1.t
        p1 = T_GLOBAL_BODY_T2.t

        qr = UnitQuaternion(qslerp(q0.vec, q1.vec, i, shortest=True), norm=True)
        pr = p0 * (1 - i) + i * p1

        T_GLOBAL_BODY_Ti = SE3.Rt(qr.R, pr, check=True)
        if not SE3.isvalid(T_GLOBAL_BODY_Ti, check=True):
            if T_GLOBAL_BODY_Ti.A is None:
                return None
            else:
                q = UnitQuaternion(SO3(T_GLOBAL_BODY_Ti.R, check=False), norm=True).unit()
                T_GLOBAL_BODY_Ti = SE3.Rt(q.R, T_GLOBAL_BODY_Ti.t, check=True)
    return T_GLOBAL_BODY_Ti


class ROSBag_TrueRelPoses:
    def __init__(self):
        pass

    @staticmethod
    def extract(bagfile_in_name,
                bagfile_out_name,
                cfg,
                stddev_pos=0.0,
                perc_outliers=0.0,
                stddev_outlier=0.5,
                use_header_timestamp=False,
                verbose=False
                ):

        if not os.path.isfile(bagfile_in_name):
            print("ROSBag_Poses2RelPose: could not find file: %s" % bagfile_in_name)
            return False
        cfg = os.path.abspath(cfg)
        if not os.path.isfile(cfg):
            print("ROSBag_Poses2RelPose: could not find file: %s" % cfg)
            return False

        if verbose:
            print("ROSBag_Poses2RelPose:")
            print("* bagfile in name: " + str(bagfile_in_name))
            print("* bagfile out name: " + str(bagfile_out_name))
            print("* cfg YAML file: \t " + str(cfg))
            print("* stddev_pos: " + str(stddev_pos))
            print("* perc_outliers: " + str(perc_outliers))
            print("* outlier_stddev: " + str(stddev_outlier))
            print("* use_header_timestamp: " + str(use_header_timestamp))

        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_in_name)
        except:
            if verbose:
                print("ROSBag_Poses2RelPose: Unexpected error!")

            return False

        ## create result dir:
        [root, ext] = os.path.splitext(bagfile_out_name)
        [head, tail] = os.path.split(root)
        try:  # else already exists
            os.makedirs(head)
        except:
            pass

        if verbose:
            print("* result_dir: \t " + str(head))

        dict_cfg = None
        with open(cfg, "r") as yamlfile:
            dict_cfg = yaml.load(yamlfile, Loader=yaml.FullLoader)
            if "sensor_positions" not in dict_cfg:
                print("[sensor_positions] does not exist in fn=" + cfg)
                return False
            if "sensor_orientations" not in dict_cfg:
                print("[sensor_orientations] does not exist in fn=" + cfg)
            if "sensor_topics" not in dict_cfg:
                print("[sensor_topics] does not exist in fn=" + cfg)
                return False
            if "relpose_topics" not in dict_cfg:
                print("[relpose_topics] does not exist in fn=" + cfg)
                return False
            print("Read successful")
        if verbose:
            print("configuration contains:")
            print("sensor_positions:" + str(dict_cfg["sensor_positions"]))
            print("sensor_orientations:" + str(dict_cfg["sensor_orientations"]))
            print("sensor_topics:" + str(dict_cfg["sensor_topics"]))
            print("relpose_topics:" + str(dict_cfg["relpose_topics"]))

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("ROSBag_Poses2RelPose: Unexpected error, bag file might be empty!")
            bag.close()
            return False

        ## create csv file according to the topic names:
        dict_file_writers = dict()
        dict_header_written = dict()
        dict_csvfile_hdls = dict()
        idx = 0

        ## check if desired topics are in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']

        # check if desired topics are in
        for id, topicName in dict_cfg["sensor_topics"].items():
            found_ = False
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found_ = True
            if not found_:
                print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")
        for id, topicName in dict_cfg["relpose_topics"].items():
            found_ = False
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found_ = True
            if not found_:
                print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")

        if verbose:
            print("\nROSBag_Poses2RelPose: num messages " + str(num_messages))

        ## store the BODY SENSOR pose in a dictionary
        dict_T_BODY_SENSOR = dict() # map<topic, SE3>
        dict_poses = dict() # map<topic<timestamp, SE3>>
        for ID, sensor_pos in dict_cfg["sensor_positions"].items():
            t_BT = np.array(sensor_pos)
            q_BT = UnitQuaternion()
            if ID in dict_cfg["sensor_orientations"].keys():
                # expecting: w,x,y,z
                q_BT = UnitQuaternion(np.array(dict_cfg["sensor_orientations"][ID]), norm=True)
                pass
            dict_T_BODY_SENSOR[dict_cfg["sensor_topics"][ID]] = SE3.Rt(q_BT.R, t_BT, check=True)
            dict_poses[dict_cfg["sensor_topics"][ID]] = dict()

        ## extract the poses of the desired topics from the BAG file
        round_decimals = 4
        try:  # else already exists
            print("ROSBag_Poses2RelPose: extracting poses...")
            cnt_poses = 0
            for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                if topic in dict_cfg["sensor_topics"].values():
                    T_GLOBAL_BODY = None
                    if hasattr(msg, 'header') and hasattr(msg, 'pose'):  # POSE_STAMPED
                        t = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                        q_GB = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                msg.pose.orientation.z]

                        q = UnitQuaternion(q_GB, norm=True)
                        T_GLOBAL_BODY = SE3.Rt(q.R, t, check=True)
                        pass
                    elif hasattr(msg, 'header') and hasattr(msg, 'transform'):
                        t = np.array(
                            [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
                        q_GB = [msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y,
                                msg.transform.rotation.z]
                        q = UnitQuaternion(q_GB, norm=True)
                        T_GLOBAL_BODY = SE3.Rt(q.R, t, check=True)
                    else:
                        print("\nROSbag_TrueRanges: unsupported message " + str(msg))
                        continue

                    if T_GLOBAL_BODY is not None:
                        timestamp = round(msg.header.stamp.to_sec(), round_decimals)
                        T_BODY_SENSOR = dict_T_BODY_SENSOR[topic]
                        dict_poses[topic][timestamp] = T_GLOBAL_BODY * T_BODY_SENSOR
                        cnt_poses = cnt_poses + 1
                        pass
                pass

            if cnt_poses == 0:
                print("\nROSbag_TrueRanges: no poses obtained!")
                return False
            else:
                print("\nROSbag_TrueRanges: poses extractd: " + str(cnt_poses))

        except AssertionError as error:
            print(error)
            print(
                "ROSBag_Poses2RelPose: Unexpected error while reading the bag file!\n * try: $ rosbag fix <bagfile> <fixed>")
            return False

        ## convert poses to History
        dict_history = dict() # map<topic, history>
        for topic, poses in dict_poses.items():
            dict_history[topic] = HistoryBuffer(dict_t=poses)

        ## TODO:
        cnt = 0;
        try:  # else already exists
            print("ROSbag_TrueRanges: computing new relative pose measurements...")
            with rosbag.Bag(bagfile_out_name, 'w') as outbag:
                for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                    if topic in dict_cfg["relpose_topics"].values() and hasattr(msg, 'poses') and hasattr(msg, 'header'):
                        ID1 = get_key_from_value(dict_cfg["relpose_topics"], topic)
                        idx_pose = 0
                        for relpose in msg.poses: # id, pose, covariance
                            ID2 = relpose.id
                            timestamp = round(msg.header.stamp.to_sec(), round_decimals)
                            for id2, topic2 in dict_cfg["sensor_topics"].items():
                                if id2 == relpose.id:
                                    #ID2 = get_key_from_value(dict_cfg["sensor_topics"], topic2)

                                    T_GLOBAL_SENSOR1 = interpolate_pose(dict_history[dict_cfg["sensor_topics"][ID1]], timestamp)
                                    T_GLOBAL_SENSOR2 = interpolate_pose(dict_history[dict_cfg["sensor_topics"][ID2]], timestamp)

                                    if T_GLOBAL_SENSOR1 is not None and T_GLOBAL_SENSOR2 is not None:
                                        T_SENSOR1_SENSOR2 = SE3(T_GLOBAL_SENSOR1 * T_GLOBAL_SENSOR2.inv())
                                        p = T_SENSOR1_SENSOR2.t
                                        q = UnitQuaternion(T_SENSOR1_SENSOR2.R)
                                        qv = q.vec

                                        # TODO: add noise etc.

                                        msg.poses[idx_pose].pose.position.x = p[0]
                                        msg.poses[idx_pose].pose.position.y = p[1]
                                        msg.poses[idx_pose].pose.position.z = p[2]
                                        msg.poses[idx_pose].pose.orientation.x = qv[0]
                                        msg.poses[idx_pose].pose.orientation.y = qv[1]
                                        msg.poses[idx_pose].pose.orientation.z = qv[2]
                                        msg.poses[idx_pose].pose.orientation.w = qv[3]
                                        if use_header_timestamp and hasattr(msg, "header"):
                                            outbag.write(topic, msg, msg.header.stamp)
                                        else:
                                            outbag.write(topic, msg, t)
                                        cnt += 1
                                        pass
                                    else:
                                        continue

                            idx_pose += 1

                    else:
                        # write other topic
                        if use_header_timestamp and hasattr(msg, "header"):
                            outbag.write(topic, msg, msg.header.stamp)
                        else:
                            outbag.write(topic, msg, t)
                        pass




        except  Exception as e:
            print("ROSBag_Poses2RelPose: Unexpected error while creating the CSV files! msg=%s" % repr(e))
            print(str(sys.exc_info()))
            return False
        ## CLEANUP:
        # close all csv files
        for key, hdls in dict_csvfile_hdls.items():
            hdls.close()

        # check if a topic was found by checking if the topic header was written
        for topic, header_writen in dict_header_written.items():
            if not header_writen:
                print("\nROSBag_Poses2RelPose: \n\tWARNING topic [" + str(topic) + "] was not in bag-file")
                print("\tbag file [" + str(bagfile_in_name) + "] contains: ")
                # print(info_dict['topics'])
                for t in info_dict['topics']:
                    print(t['topic'])
                return False

        if verbose:
            print("\nROSBag_Poses2RelPose: extracting done! ")

        bag.close()
        return True


def main():
    # example: ROSBag_Pose2Ranges.py --bagfile ../test/sample_data//uwb_calib_a01_2023-08-31-21-05-46.bag --topic /d01/mavros/vision_pose/pose --cfg ../test/sample_data/config.yaml --verbose
    parser = argparse.ArgumentParser(
        description='ROSBag_Poses2RelPose: extract given pose topics and compute for each relative poses measurement a true pose, which is stored into a CSV file')
    parser.add_argument('--bagfile_in', help='input bag file', required=True)
    parser.add_argument('--bagfile_out', help='output bag file', default="")
    parser.add_argument('--cfg',
                        help='YAML configuration file describing the setup: {rel_tag_positions, abs_anchor_positions}',
                        default="config.yaml", required=True)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--std_pos',
                        help='standard deviation of generated measurements: z = d + white_noise(std_range)',
                        default=0.1)
    parser.add_argument('--perc_outliers', help='specifies a percentage of generated outliers by modified the '
                                                'measurement: z = d + white_noise(std_range) + std_range',
                        default=0.0)
    parser.add_argument('--outlier_stddev', help='standard deviation of the outliers.',
                        default=1.0)
    parser.add_argument('--use_header_timestamp', action='store_true',
                        help='overwrites the bag time with the header time stamp', default=False)

    tp_start = time.time()
    args = parser.parse_args()

    if ROSBag_TrueRelPoses.extract(bagfile_in_name=args.bagfile_in,
                                 bagfile_out_name=args.bagfile_out,
                                 cfg=args.cfg,
                                 verbose=args.verbose,
                                 stddev_pos=float(args.std_pos),
                                 perc_outliers=float(args.perc_outliers),
                                 stddev_outlier=float(args.outlier_stddev),
                                 use_header_timestamp=args.use_header_timestamp ):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
