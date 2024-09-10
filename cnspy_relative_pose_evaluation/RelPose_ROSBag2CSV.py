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

import rosbag
import time
import os
import argparse
import yaml
import csv
from tqdm import tqdm
from numpy import linalg as LA
from spatialmath import UnitQuaternion

#from uwb_msgs.msg import TwoWayRangeStamped
from cnspy_trajectory.HistoryBuffer import get_key_from_value


class RelPose_ROSBag2CSV:
    def __init__(self):
        pass

    #
    # extract
    #
    @staticmethod
    def extract(bagfile_name, cfg, result_dir=None, fn_list=[], verbose=False, with_cov=False
                ):
        if not os.path.isfile(bagfile_name):
            print("RelPose_ROSBag2CSV: could not find file: %s" % bagfile_name)
            return False

        cfg = os.path.abspath(cfg)
        if not os.path.isfile(cfg):
            print("RelPose_ROSBag2CSV: could not find file: %s" % cfg)
            return False

        if verbose:
            print("RelPose_ROSBag2CSV:")
            print("* bagfile name: " + str(bagfile_name))
            print("* cfg: \t " + str(cfg))
            print("* filename_list: " + str(fn_list))

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
            
        topic_list = dict_cfg["relpose_topics"].values()
        if len(topic_list) < 1:
            print("RelPose_ROSBag2CSV: no topics specified!")
            return False

        if fn_list:
            if len(topic_list) != len(fn_list):
                print("RelPose_ROSBag2CSV: topic_list and fn_list must have the same length!")
                return False
        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_name)
        except:
            if verbose:
                print("RelPose_ROSBag2CSV: Unexpected error!")

            return False

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("RelPose_ROSBag2CSV: Unexpected error, bag file might be empty!")
            bag.close()
            return False

        ## create result dir:
        if result_dir is None:
            folder = str(bagfile_name).replace(".bag", "")
        else:
            folder = str(result_dir)

        folder = os.path.abspath(folder)
        try:  # else already exists
            os.makedirs(folder)
        except:
            pass

        if verbose:
            print("* result_dir: \t " + str(folder))

        ## create csv file according to the topic names:
        dict_file_writers = dict()
        dict_header_written = dict()
        dict_csvfile_hdls = dict()
        idx = 0
        for id, topicName in dict_cfg["relpose_topics"].items():

            if topicName[0] != '/':
                print("RelPose_ROSBag2CSV: Not a proper topic name: %s (should start with /)" % topicName)
                continue

            if not fn_list:
                filename = str(folder + '/') + str.replace(topicName[1:], '/', '_') + '.csv'
            else:
                fn = fn_list[idx]
                [root, ext] = os.path.splitext(fn)
                [head, tail] = os.path.split(root)
                if ext:
                    filename = str(folder + '/') + tail + ext
                else:
                    filename = str(folder + '/') + tail + '.csv'

            csvfile = open(filename, 'w+')
            file_writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
            dict_file_writers[topicName] = file_writer
            dict_header_written[topicName] = False
            dict_csvfile_hdls[topicName] = csvfile

            if verbose:
                print("RelPose_ROSBag2CSV: creating csv file: %s " % filename)

            idx = idx + 1

        ## check if desired topics are in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']
        for id, topicName in dict_cfg["relpose_topics"].items():
            found = False
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found = True

            if not found:
                print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")

        if verbose:
            print("\nRelPose_ROSBag2CSV: num messages " + str(num_messages))

        round_decimals = 4
        cnt = 0
        ## extract the desired topics from the BAG file
        for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
            if topic in dict_cfg["relpose_topics"].values():
                if hasattr(msg, 'header') and hasattr(msg, 'poses'):  # STAMPED
                    file_writer = dict_file_writers[topic]

                    if not dict_header_written[topic]:
                        file_writer.writerow(RelPose_ROSBag2CSV.get_header(with_cov))
                        dict_header_written[topic] = True

                    ID1 = get_key_from_value(dict_cfg["relpose_topics"], topic)
                    timestamp = msg.header.stamp.to_sec()
                    for pose_id in msg.poses:
                        content = RelPose_ROSBag2CSV.to_csv_line(timestamp, ID1, pose_id, round_decimals, with_cov)
                        file_writer.writerow(content)
                        cnt += 1

        ## CLEANUP:
        # close all csv files
        for topicName in topic_list:
            dict_csvfile_hdls[topicName].close()

        # check if a topic was found by checking if the topic header was written
        for topicName in topic_list:
            if not dict_header_written[topicName]:
                print("\nRelPose_ROSBag2CSV: \n\tWARNING topic [" + str(topicName) + "] was not in bag-file")
                print("\tbag file [" + str(bagfile_name) + "] contains: ")
                # print(info_dict['topics'])
                for t in info_dict['topics']:
                    print(t['topic'])
                return False

        if verbose:
            print("\nRelPose_ROSBag2CSV.extract(): extracted [%d] msgs." % cnt)

        bag.close()
        return True

    #
    # extract_to_one
    #
    @staticmethod
    def extract_to_one(bagfile_name, cfg, fn, result_dir="", ext="csv", verbose=False, with_cov=False ) -> (bool, set):
        if not os.path.isfile(bagfile_name):
            print("RelPose_ROSBag2CSV: could not find file: %s" % bagfile_name)
            return False, set()

        cfg = os.path.abspath(cfg)
        if not os.path.isfile(cfg):
            print("RelPose_ROSBag2CSV: could not find file: %s" % cfg)
            return False, set()

        if verbose:
            print("RelPose_ROSBag2CSV:")
            print("* bagfile: " + str(bagfile_name))
            print("* cfg: \t " + str(cfg))
            print("* filename: " + str(fn))

        dict_cfg = None
        with open(cfg, "r") as yamlfile:
            dict_cfg = yaml.load(yamlfile, Loader=yaml.FullLoader)
            if "relpose_topics" not in dict_cfg:
                print("[relpose_topics] does not exist in fn=" + cfg)
                return False, set()
            print("Read successful")

        if verbose:
            print("configuration contains:")
            print("relpose_topics:" + str(dict_cfg["relpose_topics"]))

        topic_list = dict_cfg["relpose_topics"].values()
        if len(topic_list) < 1:
            print("RelPose_ROSBag2CSV: no topics specified!")
            return False, set()

        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_name)
        except:
            if verbose:
                print("RelPose_ROSBag2CSV: Unexpected error!")

            return False, set()

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("RelPose_ROSBag2CSV: Unexpected error, bag file might be empty!")
            bag.close()
            return False, set()

        ## create result dir:
        if result_dir == "":
            folder = str(bagfile_name).replace(".bag", "")
        else:
            folder = result_dir

        folder = os.path.abspath(folder)
        try:  # else already exists
            os.makedirs(folder)
        except:
            pass

        if verbose:
            print("* result_dir: \t " + str(folder))

        if not fn:
          filename = str(folder + '/') + 'all_ranges.csv'
        else:
          filename = fn

        if verbose:
            print("* topic_list: \t " + str(topic_list))
            print("* filename_list: " + str(filename))

        for topicName in topic_list:
            if topicName[0] != '/':
                print("RelPose_ROSBag2CSV: Not a proper topic name: %s (should start with /)" % topicName)
                continue

        [root, ext] = os.path.splitext(filename)
        [head, tail] = os.path.split(root)
        if ext:
            filename = str(folder + '/') + tail + ext
        else:
            filename = str(folder + '/') + tail + '.csv'

        csvfile = open(filename, 'w+')
        file_writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
        file_writer.writerow(RelPose_ROSBag2CSV.get_header(with_cov))

        ## check if desired topics are in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']
        for topicName in topic_list:
            found = False
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found = True

            if not found:
                print("# WARNING: desired topic [" + str(topicName) + "] is not in bag file!")

        if verbose:
            print("\nRelPose_ROSBag2CSV: num messages " + str(num_messages))

        round_decimals = 4
        cnt = 0

        IDs = set()
        ## extract the desired topics from the BAG file
        for topic, msg, t in tqdm(bag.read_messages(), total=num_messages, unit="msgs"):
            if topic in topic_list:
                if hasattr(msg, 'header') and hasattr(msg, 'poses'):  # STAMPED
                    # HINT: conversions:

                    ID1 = get_key_from_value(dict_cfg["relpose_topics"], topic)
                    timestamp = msg.header.stamp.to_sec()
                    IDs.add(ID1)
                    for pose_id in msg.poses:
                        content = RelPose_ROSBag2CSV.to_csv_line(timestamp, ID1, pose_id, round_decimals, with_cov)
                        IDs.add(pose_id.id)
                        file_writer.writerow(content)
                        cnt += 1

        ## CLEANUP:
        csvfile.close()


        if verbose:
            print("\nRelPose_ROSBag2CSV.extract_to_one(): extracted [%d] msgs." % cnt)
            print("\nRelPose_ROSBag2CSV.extract_to_one(): IDs found: [%s]." % str(IDs))
        bag.close()
        return True, IDs

    @staticmethod
    def get_header(with_cov=False):
        if with_cov:
            return ["t", "ID1", "ID2", "tx", "ty", "tz", "qw", "qx", "qy", "qz", "range", "angle", 'pxx', 'pxy', 'pxz', 'pyy', 'pyz', 'pzz', 'qrr',
                    'qrp', 'qry', 'qpp', 'qpy', 'qyy']
        else:
            return ["t", "ID1", "ID2", "tx", "ty", "tz", "qw", "qx", "qy", "qz", "range", "angle"]

    @staticmethod
    def to_csv_line(timestamp, ID1, pose_id, round_decimals = 4, with_cov=False):
        range = LA.norm([pose_id.pose.position.x, pose_id.pose.position.y, pose_id.pose.position.z])
        q = UnitQuaternion(
            [pose_id.pose.orientation.w, pose_id.pose.orientation.x, pose_id.pose.orientation.y,
             pose_id.pose.orientation.z], norm=True).unit()
        [ang, vec] = q.angvec()
        ID2 = pose_id.id

        content = ["%f" % timestamp, str(ID1), str(ID2),
                   str(round(pose_id.pose.position.x, round_decimals)),
                   str(round(pose_id.pose.position.y, round_decimals)),
                   str(round(pose_id.pose.position.z, round_decimals)),
                   str(round(pose_id.pose.orientation.w, round_decimals)),
                   str(round(pose_id.pose.orientation.x, round_decimals)),
                   str(round(pose_id.pose.orientation.y, round_decimals)),
                   str(round(pose_id.pose.orientation.z, round_decimals)),
                   str(round(range, round_decimals)),
                   str(round(ang, round_decimals))]
        if with_cov:
            P = pose_id.covariance
            cov_tri_up = [P[0], P[1], P[2], P[7], P[8], P[14], P[21], P[22], P[23], P[28], P[29], P[35]]
            cov = []
            for i in cov_tri_up:
                cov.append(str(round(i, round_decimals)))
            content.extend(cov)

        return content

def main():
    # test3: python3 TWR_ROSbag2CS.py --bagfile ../test/example.bag --topics /a01/ranging /a02/ranging--verbose --filenames ranges.csv
    parser = argparse.ArgumentParser(
        description='RelPose_ROSBag2CSV: extract and store given topics of a rosbag into a CSV file')
    parser.add_argument('--bagfile', help='input bag file', default="not specified")
    parser.add_argument('--cfg', help='YAML configuration file describing the setup: {relpose_topics}', default="config.yaml", required=True)
    parser.add_argument('--filenames', nargs='*', help='csv filename of corresponding topic', default=[])
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--with_cov', action='store_true', default=False)
    tp_start = time.time()
    args = parser.parse_args()


    res = RelPose_ROSBag2CSV.extract(bagfile_name=args.bagfile,
                                 cfg=args.cfg,
                                 fn_list=args.filenames,
                                 result_dir=args.result_dir,
                                 verbose=args.verbose,
                                 with_cov=args.with_cov)

    if res:
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
