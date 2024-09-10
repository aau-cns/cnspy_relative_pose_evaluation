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
import rosbag
import time
import os
import argparse
import yaml
from tqdm import tqdm
import numpy as np
from enum import Enum

from spatialmath import UnitQuaternion, SE3, base

from cnspy_trajectory.ROSBag_Pose import ROSBag_Pose
from cnspy_trajectory.HistoryBuffer import get_key_from_value
from cnspy_relative_pose_evaluation.ROSBag_TrueRelPoses import interpolate_pose, random_orientation
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseTypes(Enum):
    SE2 = 'SE2'
    SE3 = 'SE3'
    Pos3DYaw = 'Pos3DYaw'

    def __str__(self):
        return self.value

    @staticmethod
    def list():
        return list([str(PoseTypes.SE2),
                     str(PoseTypes.SE3),
                     str(PoseTypes.Pos3DYaw)])


class ROSBag_Pose2AbsPoses:
    def __init__(self):
        pass

    @staticmethod
    def extract(bagfile_in,
                bagfile_out,
                cfg,
                stddev_pos=0.0,
                stddev_or=0.0,
                use_header_timestamp=False,
                verbose=False,
                replace_with_new_topic=False
                ):

        if not os.path.isfile(bagfile_in):
            print("ROSBag_Pose2AbsPoses: could not find file: %s" % bagfile_in)
            return False
        cfg = os.path.abspath(cfg)
        if not os.path.isfile(cfg):
            print("ROSBag_Pose2AbsPoses: could not find file: %s" % cfg)
            return False

        if verbose:
            print("ROSBag_Pose2AbsPoses:")
            print("* bagfile in name: " + str(bagfile_in))
            print("* bagfile out name: " + str(bagfile_out))
            print("* cfg YAML file: \t " + str(cfg))
            print("* stddev_pos: " + str(stddev_pos))
            print("* stddev_or: " + str(stddev_or))
            print("* use_header_timestamp: " + str(use_header_timestamp))
            print("* replace_with_new_topic: " + str(replace_with_new_topic))

        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_in)
        except:
            if verbose:
                print("ROSBag_Pose2AbsPoses: Unexpected error!")

            return False

        ## create result dir:
        [root, ext] = os.path.splitext(bagfile_out)
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
            if "sensor_orientations" not in dict_cfg:
                print("[sensor_orientations] does not exist in fn=" + cfg)
            if "sensor_topics" not in dict_cfg:
                print("[sensor_topics] does not exist in fn=" + cfg)
                return False
            if "true_pose_topics" not in dict_cfg:
                print("[true_pose_topics] does not exist in fn=" + cfg)
                return False
            if "new_sensor_topics" not in dict_cfg:
                print("[new_sensor_topics] does not exist in fn=" + cfg)
            if "pose_types" not in dict_cfg:
                print("[pose_types] does not exist in fn=" + cfg + " with types: " + str(PoseTypes.list()))
                return False
            print("Read successful")
        if verbose:
            print("configuration contains:")
            print("sensor_topics:" + str(dict_cfg["sensor_topics"]))
            print("true_pose_topics:" + str(dict_cfg["true_pose_topics"]))
            print("pose_types:" + str(dict_cfg["pose_types"]))
        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("ROSBag_Pose2AbsPoses: Unexpected error, bag file might be empty!")
            bag.close()
            return False

        ## check if desired topics are in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']

        # check if desired topics are in
        all_found = True
        for id, topicName in dict_cfg["sensor_topics"].items():
            found_ = False
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found_ = True
            if not found_:
                print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")
                all_found = False
        for id, topicName in dict_cfg["true_pose_topics"].items():
            found_ = False
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found_ = True
            if not found_:
                print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")
                all_found = False

        if not all_found:
            print("Topics contained:")
            for t_ in bag_topics:
                print("* " + str(t_))
        if verbose:
            print("\nROSBag_Pose2AbsPoses: num messages " + str(num_messages))

        ## store the BODY SENSOR pose in a dictionary
        dict_T_BODY_SENSOR = dict() # map<topic, SE3>+
        for ID, sensor_topic in dict_cfg["sensor_topics"].items():
            t_BT = np.zeros((3, 1))
            if "sensor_positions" in dict_cfg and ID in dict_cfg["sensor_positions"].keys():
                t_BT = np.array(dict_cfg["sensor_positions"][ID])

            q_BT = UnitQuaternion()
            if "sensor_orientations" in dict_cfg and ID in dict_cfg["sensor_orientations"].keys():
                # expecting: w,x,y,z
                q_vec = np.array(dict_cfg["sensor_orientations"][ID])
                q_BT = UnitQuaternion(q_vec).unit()
                pass
            dict_T_BODY_SENSOR[dict_cfg["sensor_topics"][ID]] = SE3.Rt(q_BT.R, t_BT, check=True)

        ## extract the poses of the desired topics from the BAG file
        round_decimals = 6
        dict_history = ROSBag_Pose.extract_poses(bag=bag, num_messages=num_messages,
                                                 dict_topic_pose_body=dict_cfg["true_pose_topics"],
                                                 dict_senor_topic_pose=dict_cfg["sensor_topics"],
                                                 round_decimals=round_decimals,
                                                 dict_T_BODY_SENSOR=dict_T_BODY_SENSOR)
        if len(dict_history) == 0:
            return False

        cnt = 0
        try:  # else already exists
            print("ROSBag_Pose2AbsPoses: computing new absolute pose measurements of a certain type at given senor_topics")
            with rosbag.Bag(bagfile_out, 'w') as outbag:
                for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                    if topic in dict_cfg["sensor_topics"].values():
                        id_topic = get_key_from_value(dict_cfg["sensor_topics"], topic)
                        timestamp = msg.header.stamp.to_sec()
                        msg_new = PoseWithCovarianceStamped()
                        msg_new.header = msg.header

                        pose_type = PoseTypes(dict_cfg["pose_types"][id_topic])
                        # check if the topic should be renamed.
                        topic_out = topic
                        if "new_sensor_topics" in dict_cfg:
                            if id_topic in dict_cfg["new_sensor_topics"]:
                                topic_out = dict_cfg["new_sensor_topics"][id_topic]

                                if not replace_with_new_topic:
                                    # store original message, otherwise it gets lost:
                                    if use_header_timestamp and hasattr(msg, "header"):
                                        outbag.write(topic, msg, msg.header.stamp)
                                    else:
                                        outbag.write(topic, msg, t)
                            else:
                                print("No new_sensor_topic with ID=[" + str(id_topic) + "] found!")
                                return False
                        T_GLOBAL_SENSOR1 = interpolate_pose(dict_history[topic], timestamp, round_decimals)
                        if T_GLOBAL_SENSOR1 is not None:
                            p = T_GLOBAL_SENSOR1.t
                            q = UnitQuaternion(T_GLOBAL_SENSOR1.R, norm=True).unit()
                            if stddev_pos > 0:
                                p_noise = np.random.normal(0, stddev_pos, size=3)
                                p = p + p_noise.T
                            if stddev_or > 0:
                                q_noise = random_orientation(stddev_rad=stddev_or)
                                q = q*q_noise
                            else:
                                pass
                            var_pos = stddev_pos * stddev_pos
                            var_or = stddev_or * stddev_or
                            if pose_type is PoseTypes.SE2:
                                rpy = base.tr2rpy(T=q.R, unit='rad', order='zyx')
                                rpy[0] = 0
                                rpy[1] = 0
                                R_ = base.rpy2r(rpy, unit='rad', order='zyx')
                                q = UnitQuaternion(R_)
                                qv = q.vec

                                msg_new.pose.pose.position.x = p[0]
                                msg_new.pose.pose.position.y = p[1]
                                msg_new.pose.pose.position.z = 0
                                msg_new.pose.pose.orientation.w = qv[0]
                                msg_new.pose.pose.orientation.x = qv[1]
                                msg_new.pose.pose.orientation.y = qv[2]
                                msg_new.pose.pose.orientation.z = qv[3]

                                if stddev_pos > 0:
                                    # upper left block
                                    for idx in [0, 7]:
                                        msg_new.pose.covariance[idx] = var_pos
                                if stddev_or > 0:
                                    # lower right block

                                    for idx in [35]:
                                        msg_new.pose.covariance[idx] = var_or
                            elif pose_type is PoseTypes.SE3:
                                qv = q.vec

                                msg_new.pose.pose.position.x = p[0]
                                msg_new.pose.pose.position.y = p[1]
                                msg_new.pose.pose.position.z = p[2]
                                msg_new.pose.pose.orientation.w = qv[0]
                                msg_new.pose.pose.orientation.x = qv[1]
                                msg_new.pose.pose.orientation.y = qv[2]
                                msg_new.pose.pose.orientation.z = qv[3]
                                if stddev_pos > 0:
                                    # upper left block
                                    var_pos = stddev_pos*stddev_pos

                                    for idx in [0, 7, 14]:
                                        msg_new.pose.covariance[idx] = var_pos
                                if stddev_or > 0:
                                    # lower right block
                                    var_or = stddev_or * stddev_or
                                    for idx in [21, 28, 35]:
                                        msg_new.pose.covariance[idx] = var_or
                            elif pose_type is PoseTypes.Pos3DYaw:
                                rpy = base.tr2rpy(T=q.R, unit='rad', order='zyx')
                                rpy[0] = 0
                                rpy[1] = 0
                                R_ = base.rpy2r(rpy, unit='rad', order='zyx')
                                q = UnitQuaternion(R_)
                                qv = q.vec

                                msg_new.pose.pose.position.x = p[0]
                                msg_new.pose.pose.position.y = p[1]
                                msg_new.pose.pose.position.z = p[2]
                                msg_new.pose.pose.orientation.w = qv[0]
                                msg_new.pose.pose.orientation.x = qv[1]
                                msg_new.pose.pose.orientation.y = qv[2]
                                msg_new.pose.pose.orientation.z = qv[3]

                                if stddev_pos > 0:
                                    # upper left block
                                    for idx in [0, 7, 14]:
                                        msg_new.pose.covariance[idx] = var_pos
                                if stddev_or > 0:
                                    # lower right block

                                    for idx in [35]:
                                        msg_new.pose.covariance[idx] = var_or
                            else:
                                print("ROSBag_Pose2AbsPoses: unsupported PoseTypes!")
                                return False
                            # for id2
                            if use_header_timestamp and hasattr(msg, "header"):
                                outbag.write(topic_out, msg_new, msg_new.header.stamp)
                            else:
                                outbag.write(topic_out, msg_new, t)
                            cnt += 1
                        pass
                    else:
                        # write other topic
                        if use_header_timestamp and hasattr(msg, "header"):
                            outbag.write(topic, msg, msg.header.stamp)
                        else:
                            outbag.write(topic, msg, t)
                        pass

        except  Exception as e:
            print("ROSBag_Pose2AbsPoses: Unexpected error while creating the bag file! msg=%s" % repr(e))
            print(str(sys.exc_info()))
            return False
        ## CLEANUP:
        except AssertionError as error:
            print(error)
            print("ROSBag_Pose2AbsPoses: Unexpected error while creating bag file")
            return False

        if verbose:
            print("\nROSBag_Pose2AbsPoses: " + str(cnt) + " measurements modified!")
        bag.close()
        return True

def main():
    # example: ROSBag_Pose2Ranges.py --bagfile ../test/sample_data//uwb_calib_a01_2023-08-31-21-05-46.bag --topic /d01/mavros/vision_pose/pose --cfg ../test/sample_data/config.yaml --verbose
    parser = argparse.ArgumentParser(
        description='ROSBag_Pose2AbsPoses: extract given pose topics and compute for each sensor topic poses a ' +
                    'absolute pose measurement (which can be perturbed) of a specific type which is added to a ' +
                    'new bag file')
    parser.add_argument('--bagfile_in', help='input bag file', required=True)
    parser.add_argument('--bagfile_out', help='output bag file', required=True)
    parser.add_argument('--cfg',
                        help='YAML configuration file describing the setup: ' +
                             '{sensor_positions:{<id>:[x,y,z], ...}, sensor_orientations:{<id>:[w,x,y,z], ...}, ' +
                             'sensor_topics:{<id>:<topic_name>, ...}, true_pose_topics:{<id>:<topic_name>, ...}, ' +
                             'new_sensor_topics:{<id>:<topic_name>, ...}, pose_types:{<id>:<SE2, SE3, or Pos3DYaw>, ...}',
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
                        help='removes sensor_topic if a new_sensor_topic was specified', default=False)


    tp_start = time.time()
    args = parser.parse_args()

    if ROSBag_Pose2AbsPoses.extract(bagfile_in=args.bagfile_in,
                                    bagfile_out=args.bagfile_out,
                                    cfg=args.cfg,
                                    verbose=args.verbose,
                                    stddev_pos=float(args.std_pos),
                                    stddev_or=float(args.std_or),
                                    use_header_timestamp=args.use_header_timestamp,
                                    replace_with_new_topic=args.replace_with_new_topic):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass