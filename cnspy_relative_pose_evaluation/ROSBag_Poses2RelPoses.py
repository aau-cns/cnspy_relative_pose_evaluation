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
import csv
from tqdm import tqdm
import numpy as np
from numpy import linalg as LA
from spatialmath import UnitQuaternion, SO3, SE3
from spatialmath.base.quaternions import qslerp
from cnspy_trajectory.HistoryBuffer import get_key_from_value
from cnspy_trajectory.ROSBag_Pose import ROSBag_Pose


def relpose_to_csv_line(timestamp, ID1, ID2, Pose, round_decimals=4):
    t = Pose.t
    range = LA.norm(t)
    q = UnitQuaternion(Pose.R)
    qv = q.vec
    [ang, vec] = q.angvec()
    content = ["%f" % round(timestamp, round_decimals), str(ID1), str(ID2),
               str(round(t[0], round_decimals)),
               str(round(t[1], round_decimals)),
               str(round(t[2], round_decimals)),
               str(round(qv[3], round_decimals)),
               str(round(qv[0], round_decimals)),
               str(round(qv[1], round_decimals)),
               str(round(qv[2], round_decimals)),
               str(round(range, round_decimals)),
               str(round(ang, round_decimals))]
    return content



class ROSBag_Poses2RelPoses:
    def __init__(self):
        pass

    @staticmethod
    def extract(bagfile_name, cfg, filename, result_dir="", verbose=False):

        if not os.path.isfile(bagfile_name):
            print("ROSBag_Poses2RelPoses: could not find file: %s" % bagfile_name)
            return False
        cfg = os.path.abspath(cfg)
        if not os.path.isfile(cfg):
            print("ROSBag_Poses2RelPoses: could not find file: %s" % cfg)
            return False

        if verbose:
            print("ROSBag_Poses2RelPoses:")
            print("* bagfile name: " + str(bagfile_name))
            print("* cfg: \t " + str(cfg))
            print("* filename: " + str(filename))

        ## Open BAG file:
        try:
            bag = rosbag.Bag(bagfile_name)
        except:
            if verbose:
                print("ROSBag_Poses2RelPoses: Unexpected error!")

            return False

        dict_cfg = None
        with open(cfg, "r") as yamlfile:
            dict_cfg = yaml.load(yamlfile, Loader=yaml.FullLoader)
            if "sensor_positions" not in dict_cfg:
                print("[sensor_positions] does not exist in fn=" + cfg)
                return False
            if "sensor_orientations" not in dict_cfg:
                print("[sensor_orientations] does not exist in fn=" + cfg)
            if "true_pose_topics" not in dict_cfg:
                print("[true_pose_topics] does not exist in fn=" + cfg)
                return False
            print("Read successful")
        if verbose:
            print("configuration contains:")
            print("sensor_positions:" + str(dict_cfg["sensor_positions"]))
            print("sensor_orientations:" + str(dict_cfg["sensor_orientations"]))
            print("true_pose_topics:" + str(dict_cfg["true_pose_topics"]))

        info_dict = yaml.load(bag._get_yaml_info(), Loader=yaml.FullLoader)

        if info_dict is None or 'messages' not in info_dict:
            if verbose:
                print("ROSBag_Poses2RelPoses: Unexpected error, bag file might be empty!")
            bag.close()
            return False

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

        ## create csv file according to the topic names:
        dict_file_writers = dict()
        dict_header_written = dict()
        dict_csvfile_hdls = dict()
        idx = 0

        ## create a file writer for each sensor topic
        for  id, topicName in dict_cfg["true_pose_topics"].items():
            if filename is None:
                fn = str(folder + '/') + str.replace(topicName[1:], '/', '_') + '_sensor_' + str(id) + '.csv'
            else:
                fn = str(folder + '/') + str(filename) + '_' + str(id) + '.csv'

            csvfile = open(fn, 'w+')
            file_writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
            dict_file_writers[topicName] = file_writer
            dict_header_written[topicName] = False
            dict_csvfile_hdls[topicName] = csvfile

            if verbose:
                print("ROSbag_Pose2Ranges: creating csv file: %s " % fn)

        ## check if desired topics are in the bag file:
        num_messages = info_dict['messages']
        bag_topics = info_dict['topics']

        found = True
        for id, topicName in dict_cfg["true_pose_topics"].items():
            found_ = False
            for topic_info in bag_topics:
                if topic_info['topic'] == topicName:
                    found_ = True
                    pass
                pass
            if not found_:
                print("# ERROR: desired topic [" + str(topicName) + "] is not in bag file!")

            found = found & found_
            pass
        if not found:
            print("\nROSBag_Poses2RelPoses: not all topics found! Stopping here...")
            for topicName in dict_cfg["true_pose_topics"]:
                print(" * " + topicName)
            return False

        if verbose:
            print("\nROSBag_Poses2RelPoses: num messages " + str(num_messages))

        ## store the BODY SENSOR pose in a dictionary
        dict_T_BODY_SENSOR = dict() # map<topic, SE3>
        for ID, sensor_pos in dict_cfg["sensor_positions"].items():
            t_BT = np.array(sensor_pos)
            q_BT = UnitQuaternion()
            if ID in dict_cfg["sensor_orientations"].keys():
                # expecting: w,x,y,z
                q_BT = UnitQuaternion(np.array(dict_cfg["sensor_orientations"][ID]), norm=True)
                pass
            dict_T_BODY_SENSOR[dict_cfg["true_pose_topics"][ID]] = SE3.Rt(q_BT.R, t_BT, check=True)


        ## extract the poses of the desired topics from the BAG file
        round_decimals = 4
        dict_history = ROSBag_Pose.extract_poses(bag=bag, num_messages=num_messages,
                                                 dict_topic_pose_body=dict_cfg["true_pose_topics"],
                                                 dict_senor_topic_pose=dict_cfg["true_pose_topics"],
                                                 round_decimals=round_decimals,
                                                 dict_T_BODY_SENSOR=dict_T_BODY_SENSOR)

        ## TODO:
        try:  # else already exists
            for topic, hist in dict_history.items():
                if verbose:
                    print("\nROSBag_Poses2RelPoses: compute relative pose with respect to topic=[" + topic + "]...")
                for idx in tqdm(range(0, len(hist.t_vec)), total= len(hist.t_vec), unit="poses"):
                    file_writer = dict_file_writers[topic]
                    ID1 = get_key_from_value(dict_cfg["true_pose_topics"], topic)
                    T_GLOBAL_SENSOR1 = hist.val_vec[idx]
                    timestamp = hist.t_vec[idx]
                    for ID2, topic2 in dict_cfg["true_pose_topics"].items():
                        if topic != topic2:
                            #ID2 = get_key_from_value(dict_cfg["true_pose_topics"], topic2)

                            T_GLOBAL_SENSOR2 = None
                            if  dict_history[topic2].exists_at_t(timestamp) is None:
                                [t1, T_GLOBAL_BODY_T1] = dict_history[topic2].get_before_t(timestamp)
                                [t2, T_GLOBAL_BODY_T2] = dict_history[topic2].get_after_t(timestamp)

                                if t1 is None or t2 is None:
                                    #if verbose:
                                    #    print("* skip measurement from topic=" + topic + " at t=" + str(timestamp))
                                    continue
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


                                T_GLOBAL_SENSOR2 = SE3.Rt(qr.R, pr, check=True)*dict_T_BODY_SENSOR[topic2]
                                if not SE3.isvalid(T_GLOBAL_SENSOR2, check=True):
                                    if T_GLOBAL_SENSOR2.A is None:
                                        if verbose:
                                            print(
                                                "* interp failed: skip measurement from topic=" + topic + " at t=" + str(
                                                    timestamp))
                                        continue
                                    else:
                                        q = UnitQuaternion(SO3(T_GLOBAL_SENSOR2.R, check=False), norm=True).unit()
                                        T_GLOBAL_SENSOR2 = SE3.Rt(q.R, T_GLOBAL_SENSOR2.t, check=True)
                                pass
                            else:
                                T_GLOBAL_SENSOR2 = dict_history[topic2].get_at_t(timestamp)*dict_T_BODY_SENSOR[topic2]

                            if not dict_header_written[topic]:
                                file_writer.writerow(["t", "ID1", "ID2", "tx", "ty", "tz", "qw", "qx", "qy", "qz", "range", "angle"])
                                dict_header_written[topic] = True

                            T_SENSOR1_SENSOR2 = SE3(T_GLOBAL_SENSOR1.inv() * T_GLOBAL_SENSOR2)
                            content = relpose_to_csv_line(timestamp, ID1, ID2, T_SENSOR1_SENSOR2, round_decimals)
                            file_writer.writerow(content)
        except  Exception as e:
            print("ROSBag_Poses2RelPoses: Unexpected error while creating the CSV files! msg=%s" % repr(e))
            print(str(sys.exc_info()))
            return False
        ## CLEANUP:
        # close all csv files
        for key, hdls in dict_csvfile_hdls.items():
            hdls.close()

        # check if a topic was found by checking if the topic header was written
        for topic, header_writen in dict_header_written.items():
            if not header_writen:
                print("\nROSBag_Poses2RelPoses: \n\tWARNING topic [" + str(topic) + "] was not in bag-file")
                print("\tbag file [" + str(bagfile_name) + "] contains: ")
                # print(info_dict['topics'])
                for t in info_dict['topics']:
                    print(t['topic'])
                return False

        if verbose:
            print("\nROSBag_Poses2RelPoses: extracting done! ")

        bag.close()
        return True


def main():
    # example: ROSBag_Pose2Ranges.py --bagfile ../test/sample_data//uwb_calib_a01_2023-08-31-21-05-46.bag --topic /d01/mavros/vision_pose/pose --cfg ../test/sample_data/config.yaml --verbose
    parser = argparse.ArgumentParser(
        description='ROSBag_Poses2RelPoses: extract given pose topics and computes for each pose a relative poses, which is stored into a CSV file')
    parser.add_argument('--bagfile', help='input bag file', required=True)
    parser.add_argument('--cfg', help='YAML configuration file describing the setup: {sensor_positions, sensor_topics}', default="config.yaml", required=True)
    parser.add_argument('--filename', help='csv filename of corresponding topic', default="")
    parser.add_argument('--result_dir', help='directory to store results [otherwise bagfile name will be a directory]',
                        default='')
    parser.add_argument('--verbose', action='store_true', default=False)

    tp_start = time.time()
    args = parser.parse_args()

    if ROSBag_Poses2RelPoses.extract(bagfile_name=args.bagfile, topic=str(args.topic), cfg=args.cfg,
                                     filename=args.filename, result_dir=args.result_dir,
                                     verbose=args.verbose
                                     ):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
    pass


if __name__ == "__main__":
    main()
    pass
