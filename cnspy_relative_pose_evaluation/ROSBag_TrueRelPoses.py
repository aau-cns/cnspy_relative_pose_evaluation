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
import math
import sys

import rosbag
import time
import os
import argparse
import yaml
from tqdm import tqdm
import numpy as np
from spatialmath import UnitQuaternion, SO3, SE3
from spatialmath.base.quaternions import qslerp

from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_trajectory.BsplineSE3 import BsplineSE3, TrajectoryInterpolationType
from cnspy_trajectory.HistoryBuffer import get_key_from_value
from cnspy_trajectory.ROSBag_Pose import ROSBag_Pose
from cnspy_trajectory.SpatialConverter import SpatialConverter

from mrs_msgs.msg import PoseWithCovarianceArrayStamped, PoseWithCovarianceIdentified


def interpolate_pose(pose_hist, timestamp, round_decimals=4) -> SE3:
    if pose_hist is None:
        return None

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


def random_orientation(stddev_rad):
    ax = np.random.randn(3, 1)
    ax = ax / np.linalg.norm(ax)
    ang = np.random.randn(1, 1)[0] * stddev_rad
    return axang2quat(ax, ang[0])


def axang2quat(u, phi):
    u = u / np.linalg.norm(u)
    q_w = math.cos(phi * 0.5)
    q_v = u * math.sin(phi * 0.5)
    return UnitQuaternion(s=q_w, v=np.array(q_v)).unit()


class ROSBag_TrueRelPoses:
    def __init__(self):
        pass

    @staticmethod
    def load_dict_cfg(cfg_fn, ignore_new_topic_name, verbose) -> dict:
        with open(cfg_fn, "r") as yamlfile:
            dict_cfg = yaml.load(yamlfile, Loader=yaml.FullLoader)
            if "sensor_positions" not in dict_cfg:
                print("[sensor_positions] does not exist in fn=" + cfg_fn)
                return None
            if "sensor_orientations" not in dict_cfg:
                print("[sensor_orientations] does not exist in fn=" + cfg_fn)
            if "object_positions" not in dict_cfg:
                print("[object_positions] does not exist in fn=" + cfg_fn)
                # check if object_positions ids are different from sensor_positions
                for key, val in dict_cfg["object_postions"].items():
                    if key in dict_cfg["sensor_positions"].keys():
                        print("[object_positions] id " + str(key) + " is not unique!")
                        return None
            if "object_orientations" not in dict_cfg:
                print("[object_orientations] does not exist in fn=" + cfg_fn)
                for key, val in dict_cfg["object_orientations"].items():
                    if key in dict_cfg["sensor_positions"].keys():
                        print("[object_orientations] id " + str(key) + " is not unique!")
                        return None
            if "true_pose_topics" not in dict_cfg:
                print("[true_pose_topics] does not exist in fn=" + cfg_fn)
                return None
            if "relpose_topics" not in dict_cfg:
                print("[relpose_topics] does not exist in fn=" + cfg_fn)
                return None
            if "new_relpose_topics" not in dict_cfg:
                print("[new_relpose_topics] does not exist in fn=" + cfg_fn)
            elif ignore_new_topic_name:
                print("[new_relpose_topics] WILL BE IGNORED, but was specified in fn=" + cfg_fn)

            for key, val in dict_cfg["true_pose_topics"].items():
                if key not in dict_cfg["relpose_topics"].keys() or key not in dict_cfg["sensor_positions"].keys():
                    print("[true_pose_topics] id " + str(key) + " is not present in relpose_topics or sensor_positions!")
                    return None

            print("Configuration read successfully")
        if verbose:
            print("configuration contains:")
            print("sensor_positions:" + str(dict_cfg["sensor_positions"]))
            print("sensor_orientations:" + str(dict_cfg["sensor_orientations"]))
            print("object_positions:" + str(dict_cfg["object_positions"]))
            print("object_orientations:" + str(dict_cfg["object_orientations"]))
            print("true_pose_topics:" + str(dict_cfg["true_pose_topics"]))
            print("relpose_topics:" + str(dict_cfg["relpose_topics"]))
            if "new_relpose_topics" in dict_cfg and not ignore_new_topic_name:
                print("new_relpose_topics:" + str(dict_cfg["new_relpose_topics"]))

        return dict_cfg

    @staticmethod
    def add_noise(p, R, stddev_pos:float, stddev_or:float, pose_error_type:EstimationErrorType):
        if stddev_pos == 0 and stddev_or == 0:
            return p, R

        if stddev_pos > 0:
            n_p = np.random.normal(0, stddev_pos, size=3)
        else:
            n_p = np.zeros(3)

        if stddev_or > 0:
            n_R = np.random.normal(0, stddev_or, size=3)
        else:
            n_R = np.zeros(3)

        if pose_error_type == EstimationErrorType.type5:
            if stddev_pos > 0:
                p = p + n_p.T
            if stddev_or > 0:
                q_noise = random_orientation(stddev_rad=stddev_or)
                R = np.matmul(R, q_noise.R)
            return p, R

        elif pose_error_type == EstimationErrorType.type1 or pose_error_type == EstimationErrorType.type2:
            n_T = np.zeros(6)
            n_T[0:3] = n_p
            n_T[3:6] = n_R
            T_n = SpatialConverter.theta_to_T(n_T)
            T = np.identity(4)
            T[0:3, 0:3] = R
            T[:3, 3] = p
            if pose_error_type == EstimationErrorType.type1:
                T_AB = SE3(np.matmul(T, T_n), check=False)
                return T_AB.t, T_AB.R
            else:
                T_AB = SE3(np.matmul(T_n, T), check=False)
                return T_AB.t, T_AB.R
        else:
            assert False, "EstimationErrorType not supported!"
            return p, R

    @staticmethod
    def extract(bagfile_in_name,
                bagfile_out_name,
                cfg,
                stddev_pos=0.0,
                stddev_or=0.0,
                use_header_timestamp=False,
                verbose=False,
                replace_with_new_topic=False,
                ignore_new_topic_name=False,
                interp_type=TrajectoryInterpolationType.cubic,
                min_dt=0.05,
                pose_error_type: EstimationErrorType = EstimationErrorType.type5
                ):

        if not os.path.isfile(bagfile_in_name):
            print("ROSBag_TrueRelPoses: could not find file: %s" % bagfile_in_name)
            return False
        cfg = os.path.abspath(cfg)
        if not os.path.isfile(cfg):
            print("ROSBag_TrueRelPoses: could not find file: %s" % cfg)
            return False

        if verbose:
            print("ROSBag_TrueRelPoses:")
            print("* bagfile in name: " + str(bagfile_in_name))
            print("* bagfile out name: " + str(bagfile_out_name))
            print("* cfg YAML file: \t " + str(cfg))
            print("* stddev_pos: " + str(stddev_pos))
            print("* stddev_or: " + str(stddev_or))
            print("* use_header_timestamp: " + str(use_header_timestamp))
            print("* replace_with_new_topic: " + str(replace_with_new_topic))

        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_in_name)
        except:
            if verbose:
                print("ROSBag_TrueRelPoses: Unexpected error!")

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

        dict_cfg = ROSBag_TrueRelPoses.load_dict_cfg(cfg_fn=cfg,
                                                     ignore_new_topic_name=ignore_new_topic_name,
                                                     verbose=verbose)
        if dict_cfg is None:
            print(" ERROR configuration is wrong!")
            return False

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("ROSBag_TrueRelPoses: Unexpected error, bag file might be empty!")
            bag.close()
            return False

        ## check if desired topics are in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']

        found = ROSBag_TrueRelPoses.check_topics(bag_topics, dict_cfg, num_messages, verbose)
        if verbose and not found:
            print("ROSBag_TrueRelPoses: desired topics not found!")

        dict_static_objects = ROSBag_TrueRelPoses.load_dict_static_objects(dict_cfg)
        round_decimals = 4
        dict_bsplines, dict_history = ROSBag_TrueRelPoses.load_dict_splines(bag, dict_cfg, interp_type, min_dt,
                                                                            num_messages, round_decimals)


        if len(dict_bsplines) == 0:
            if verbose:
                print("ROSBag_TrueRelPoses: No poses found!")
            bag.close()
            return False

        cnt = 0
        try:  # else already exists
            print("ROSBag_TrueRelPoses: computing new relative pose measurements...")
            with rosbag.Bag(bagfile_out_name, 'w') as outbag:
                for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                    if topic in dict_cfg["relpose_topics"].values() and hasattr(msg, 'poses') and hasattr(msg,
                                                                                                          'header'):
                        ID1 = get_key_from_value(dict_cfg["relpose_topics"], topic)
                        idx_pose = 0
                        timestamp = msg.header.stamp.to_sec()

                        msg_gt = PoseWithCovarianceArrayStamped()
                        msg_gt.header = msg.header

                        for relpose in msg.poses:  # id, pose, covariance
                            ID2 = relpose.id

                            is_sensor = (ID2 in dict_cfg["true_pose_topics"].keys())
                            is_object = (ID2 in dict_cfg["object_positions"].keys())
                            if is_sensor or is_object:
                                # ID2 = get_key_from_value(dict_cfg["true_pose_topics"], topic2)

                                T_GLOBAL_SENSOR1 = dict_bsplines[dict_cfg["true_pose_topics"][ID1]].get_pose(
                                    t=timestamp,
                                    interp_type=interp_type,
                                    round_decimals=round_decimals)

                                if is_sensor:
                                    T_GLOBAL_SENSOR2 = dict_bsplines[dict_cfg["true_pose_topics"][ID2]].get_pose(
                                        t=timestamp,
                                        interp_type=interp_type,
                                        round_decimals=round_decimals)
                                else:
                                    T_GLOBAL_SENSOR2 = dict_static_objects[ID2]

                                if T_GLOBAL_SENSOR1 is not None and T_GLOBAL_SENSOR2 is not None:
                                    T_SENSOR1_SENSOR2 = T_GLOBAL_SENSOR1.inv() * T_GLOBAL_SENSOR2

                                    t_, R_ = ROSBag_TrueRelPoses.add_noise(p=T_SENSOR1_SENSOR2.t,
                                                                           R=T_SENSOR1_SENSOR2.R,
                                                                           stddev_pos=stddev_pos,
                                                                           stddev_or=stddev_or,
                                                                           pose_error_type=pose_error_type)
                                    p = t_
                                    q = UnitQuaternion(R_, norm=True).unit()
                                    qv = q.vec

                                    pos_id = PoseWithCovarianceIdentified()
                                    pos_id.id = ID2
                                    pos_id.pose.position.x = p[0]
                                    pos_id.pose.position.y = p[1]
                                    pos_id.pose.position.z = p[2]
                                    pos_id.pose.orientation.w = qv[0]
                                    pos_id.pose.orientation.x = qv[1]
                                    pos_id.pose.orientation.y = qv[2]
                                    pos_id.pose.orientation.z = qv[3]

                                    if stddev_pos > 0:
                                        # upper left block
                                        var_pos = stddev_pos * stddev_pos
                                        for idx in [0, 7, 14]:
                                            pos_id.covariance[idx] = var_pos
                                    if stddev_or > 0:
                                        # lower right block
                                        var_or = stddev_or * stddev_or
                                        for idx in [21, 28, 35]:
                                            pos_id.covariance[idx] = var_or
                                    msg_gt.poses.append(pos_id)
                            idx_pose += 1
                        # for id2
                        topic_out = topic
                        if "new_relpose_topics" in dict_cfg and not ignore_new_topic_name:
                            id_topic = ID1
                            if id_topic in dict_cfg["new_relpose_topics"]:
                                # assign new topic name
                                topic_out = dict_cfg["new_relpose_topics"][id_topic]
                                if not replace_with_new_topic:
                                    # store original message, otherwise it gets lost:
                                    if use_header_timestamp and hasattr(msg, "header"):
                                        outbag.write(topic, msg, msg.header.stamp)
                                    else:
                                        outbag.write(topic, msg, t)
                            else:
                                print("No new_relpose_topics with ID=[" + str(id_topic) + "] found!")
                                return False

                        if use_header_timestamp and hasattr(msg, "header"):
                            outbag.write(topic_out, msg_gt, msg.header.stamp)
                        else:
                            outbag.write(topic_out, msg_gt, t)
                        cnt += 1

                    else:
                        # write other topic
                        if use_header_timestamp and hasattr(msg, "header"):
                            outbag.write(topic, msg, msg.header.stamp)
                        else:
                            outbag.write(topic, msg, t)
                        pass




        except  Exception as e:
            print("ROSBag_TrueRelPoses: Unexpected error while creating the bag file! msg=%s" % repr(e))
            print(str(sys.exc_info()))
            return False
        ## CLEANUP:
        except AssertionError as error:
            print(error)
            print("ROSBag_TrueRelPoses: Unexpected error while creating bag file")
            return False

        if verbose:
            print("\nROSBag_TrueRelPoses: " + str(cnt) + " range measurements modified!")
        bag.close()
        return True

    @staticmethod
    def check_topics(bag_topics, dict_cfg, num_messages, verbose) -> bool:
        # check if desired topics are in
        found = True
        for ID, topicName in dict_cfg["true_pose_topics"].items():
            found_ = False
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found_ = True
            if not found_:
                print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")
                found = False
        for ID, topicName in dict_cfg["relpose_topics"].items():
            found_ = False
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found_ = True
            if not found_:
                print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")
                found = False
        if verbose:
            print("\nROSBag_TrueRelPoses: num messages " + str(num_messages))
        return found

    @staticmethod
    def load_dict_static_objects(dict_cfg):
        dict_T_GLOBAL_OBJECT = dict()  # map<topic, SE3>
        for ID, sensor_pos in dict_cfg["object_positions"].items():
            t_BT = np.array(sensor_pos)
            q_BT = UnitQuaternion()
            if ID in dict_cfg["object_orientations"].keys():
                # expecting: w,x,y,z
                q_vec = np.array(dict_cfg["object_orientations"][ID])
                q_BT = UnitQuaternion(q_vec).unit()
                pass
            dict_T_GLOBAL_OBJECT[ID] = SE3.Rt(q_BT.R, t_BT, check=True)

        return dict_T_GLOBAL_OBJECT

    @staticmethod
    def load_dict_splines(bag, dict_cfg, interp_type, min_dt=0.05, num_messages=None, round_decimals=4):
        if num_messages is None:
            info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)
            num_messages = info_dict['messages']

        ## store the BODY SENSOR pose in a dictionary
        dict_T_BODY_SENSOR = dict()  # map<topic, SE3>
        for ID, sensor_pos in dict_cfg["sensor_positions"].items():
            t_BT = np.array(sensor_pos)
            q_BT = UnitQuaternion()
            if ID in dict_cfg["sensor_orientations"].keys():
                # expecting: w,x,y,z
                q_vec = np.array(dict_cfg["sensor_orientations"][ID])
                q_BT = UnitQuaternion(q_vec).unit()
                pass
            dict_T_BODY_SENSOR[dict_cfg["true_pose_topics"][ID]] = SE3.Rt(q_BT.R, t_BT, check=True)
        ## extract the poses of the desired topics from the BAG file

        # map<true_pose_topic,History<timestamp, SE3>>
        dict_history = ROSBag_Pose.extract_poses(bag=bag,
                                                 dict_topic_pose_body=dict_cfg["true_pose_topics"],
                                                 dict_senor_topic_pose=dict_cfg["true_pose_topics"],
                                                 num_messages=num_messages,
                                                 round_decimals=round_decimals,
                                                 dict_T_BODY_SENSOR=dict_T_BODY_SENSOR)
        dict_bsplines = dict()
        for true_pose_topic, hist in dict_history.items():
            if hist:
                bspline = BsplineSE3()
                if interp_type == TrajectoryInterpolationType.cubic:
                    bspline.feed_pose_history(hist_pose=hist, min_dt=min_dt, round_decimals=round_decimals)
                elif interp_type == TrajectoryInterpolationType.linear:
                    bspline.feed_pose_history(hist_pose=hist, uniform_timestamps=False)
                dict_bsplines[true_pose_topic] = bspline
        return dict_bsplines, dict_history


def main():
    # example: ROSBag_Pose2Ranges.py --bagfile ../test/sample_data//uwb_calib_a01_2023-08-31-21-05-46.bag --topic /d01/mavros/vision_pose/pose --cfg ../test/sample_data/config.yaml --verbose
    parser = argparse.ArgumentParser(
        description='ROSBag_TrueRelPoses: extract given pose topics and compute for each relative poses measurement a' +
                    'true relative pose (which can be perturbed), which overwrites to original message or is stored ' +
                    'under a new topic in the specified bag file.')
    parser.add_argument('--bagfile_in', help='input bag file', required=True)
    parser.add_argument('--bagfile_out', help='output bag file', required=True)
    parser.add_argument('--cfg',
                        help='YAML configuration file describing the setup: ' +
                             '{sensor_positions:{<id>:[x,y,z], ...}, sensor_orientations:{<id>:[w,x,y,z], ...}, ' +
                             'relpose_topics:{<id>:<topic_name>, ...}, true_pose_topics:{<id>:<topic_name>, ...},' +
                             ' new_relpose_topics:{<id>:<topic_name>, ...}}',
                        default="config.yaml", required=True)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--std_pos',
                        help='standard deviation of generated measurements: z_p = p + white_noise(std_p)',
                        default=0.0)
    parser.add_argument('--std_or',
                        help='standard deviation of generated measurements: z_R = R *R(white_noise(std_or))',
                        default=0.0)
    parser.add_argument('--use_header_timestamp', action='store_true',
                        help='overwrites the bag time with the header time stamp', default=False)
    parser.add_argument('--replace_with_new_topic', action='store_true',
                        help='removes relpose_topics if a new_relpose_topics was specified', default=False)
    parser.add_argument('--interpolation_type', help='Trajectory interpolation type',
                        choices=TrajectoryInterpolationType.list(),
                        default=str(TrajectoryInterpolationType.linear))
    parser.add_argument('--min_dt',
                        help='temporal displacement of cubic spline control points',
                        default=0.05)
    parser.add_argument('--pose_error_type', help='Covariance perturbation type (space) of relative pose measurements',
                        choices=EstimationErrorType.list(),
                        default=str(EstimationErrorType.type5))
    tp_start = time.time()
    args = parser.parse_args()

    if ROSBag_TrueRelPoses.extract(bagfile_in_name=args.bagfile_in,
                                   bagfile_out_name=args.bagfile_out,
                                   cfg=args.cfg,
                                   verbose=args.verbose,
                                   stddev_pos=float(args.std_pos),
                                   stddev_or=float(args.std_or),
                                   use_header_timestamp=args.use_header_timestamp,
                                   interp_type=TrajectoryInterpolationType(args.interpolation_type),
                                   min_dt=float(args.min_dt),
                                   pose_error_type=EstimationErrorType(args.pose_error_type)):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
