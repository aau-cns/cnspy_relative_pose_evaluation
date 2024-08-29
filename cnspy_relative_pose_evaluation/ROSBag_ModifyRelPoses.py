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
from cnspy_trajectory.HistoryBuffer import get_key_from_value

from mrs_msgs.msg import PoseWithCovarianceArrayStamped, PoseWithCovarianceIdentified




class ROSBag_ModifyRelPoses:
    def __init__(self):
        pass

    @staticmethod
    def extract(bagfile_in_name,
                bagfile_out_name,
                cfg,
                use_header_timestamp=False,
                verbose=False,
                use_position=True,
                use_orientation=True,
                ):

        if not os.path.isfile(bagfile_in_name):
            print("ROSBag_ModifyRelPoses: could not find file: %s" % bagfile_in_name)
            return False
        cfg = os.path.abspath(cfg)
        if not os.path.isfile(cfg):
            print("ROSBag_ModifyRelPoses: could not find file: %s" % cfg)
            return False

        if verbose:
            print("ROSBag_ModifyRelPoses:")
            print("* bagfile in name: " + str(bagfile_in_name))
            print("* bagfile out name: " + str(bagfile_out_name))
            print("* cfg YAML file: \t " + str(cfg))
            print("* use_position: " + str(use_position))
            print("* use_orientation: " + str(use_orientation))
            print("* use_header_timestamp: " + str(use_header_timestamp))

        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_in_name)
        except:
            if verbose:
                print("ROSBag_ModifyRelPoses: Unexpected error!")

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
            if "relpose_topics" not in dict_cfg:
                print("[relpose_topics] does not exist in fn=" + cfg)
                return False
            print("Read successful")
        if verbose:
            print("configuration contains:")
            print("relpose_topics:" + str(dict_cfg["relpose_topics"]))

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("ROSBag_ModifyRelPoses: Unexpected error, bag file might be empty!")
            bag.close()
            return False

        ## check if desired topics are in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']

        # check if desired topics are in
        for id, topicName in dict_cfg["relpose_topics"].items():
            found_ = False
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found_ = True
            if not found_:
                print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")

        ## TODO:
        cnt = 0
        try:  # else already exists
            print("ROSBag_ModifyRelPoses: modifying relative pose measurements...")
            with rosbag.Bag(bagfile_out_name, 'w') as outbag:
                for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
                    if topic in dict_cfg["relpose_topics"].values() and hasattr(msg, 'poses') and hasattr(msg, 'header'):
                        ID1 = get_key_from_value(dict_cfg["relpose_topics"], topic)
                        idx_pose = 0
                        timestamp = msg.header.stamp.to_sec()

                        msg_gt = PoseWithCovarianceArrayStamped()
                        msg_gt.header = msg.header

                        for relpose in msg.poses: # id, pose, covariance
                            ID2 = relpose.id
                            pos_id = PoseWithCovarianceIdentified()
                            pos_id.id = ID2
                            # TODO: add noise etc.
                            if use_position:
                                pos_id.pose.position = relpose.pose.position
                            else:
                                pos_id.pose.position.x = 0
                                pos_id.pose.position.y = 0
                                pos_id.pose.position.z = 0

                            if use_orientation:
                                pos_id.pose.orientation = relpose.pose.orientation
                            else:
                                pos_id.pose.orientation.w = 1
                                pos_id.pose.orientation.x = 0
                                pos_id.pose.orientation.y = 0
                                pos_id.pose.orientation.z = 0

                            # copy covariance
                            if use_position and use_orientation:
                                pos_id.covariance = relpose.covariance
                            elif use_position:
                                # upper left block
                                for idx in [0,1,2,6,7,8,12,13,14]:
                                    pos_id.covariance[idx] = relpose.covariance[idx]
                            elif use_orientation:
                                # lower right block
                                for idx in [21,22,23,27,28,29,33,34,35]:
                                    pos_id.covariance[idx] = relpose.covariance[idx]

                            msg_gt.poses.append(pos_id)
                            idx_pose += 1
                        # for id2
                        if use_header_timestamp and hasattr(msg, "header"):
                            outbag.write(topic, msg_gt, msg.header.stamp)
                        else:
                            outbag.write(topic, msg_gt, t)
                        cnt += 1

                    else:
                        # write other topic
                        if use_header_timestamp and hasattr(msg, "header"):
                            outbag.write(topic, msg, msg.header.stamp)
                        else:
                            outbag.write(topic, msg, t)
                        pass




        except  Exception as e:
            print("ROSBag_ModifyRelPoses: Unexpected error while creating the bag file! msg=%s" % repr(e))
            print(str(sys.exc_info()))
            return False
        ## CLEANUP:
        except AssertionError as error:
            print(error)
            print("ROSBag_ModifyRelPoses: Unexpected error while creating bag file")
            return False

        if verbose:
            print("\nROSBag_ModifyRelPoses: " + str(cnt) + " range measurements modified!")
        bag.close()
        return True



def main():
    # example: ROSBag_Pose2Ranges.py --bagfile ../test/sample_data//uwb_calib_a01_2023-08-31-21-05-46.bag --topic /d01/mavros/vision_pose/pose --cfg ../test/sample_data/config.yaml --verbose
    parser = argparse.ArgumentParser(
        description='ROSBag_ModifyRelPoses: extract given relative pose topics and modifies them (removing position or orientation')
    parser.add_argument('--bagfile_in', help='input bag file', required=True)
    parser.add_argument('--bagfile_out', help='output bag file', required=True)
    parser.add_argument('--cfg',
                        help='YAML configuration file describing the setup: {rel_tag_positions, abs_anchor_positions}',
                        default="config.yaml", required=True)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--use_header_timestamp', action='store_true',
                        help='overwrites the bag time with the header time stamp', default=False)
    parser.add_argument('--remove_position', action='store_true',
                        help='clears the position of the UVDAR measurement', default=False)
    parser.add_argument('--remove_orientation', action='store_true',
                        help='clears the orientation of the UVDAR measurement', default=False)
    tp_start = time.time()
    args = parser.parse_args()

    if ROSBag_ModifyRelPoses.extract(bagfile_in_name=args.bagfile_in,
                                 bagfile_out_name=args.bagfile_out,
                                 cfg=args.cfg,
                                 verbose=args.verbose,
                                 use_header_timestamp=args.use_header_timestamp,
                                 use_position=not args.remove_position,
                                 use_orientation=not args.remove_orientation):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
