#!/usr/bin/env python3

import rosbag
import numpy as np
import pandas as pd
import argparse
from os.path import join, isfile, exists
from os import makedirs
import glob
from tqdm import tqdm


def extract_gps_msg(gps_msgs, timestamp):
    """Extract the closest NavSatFix message to a given timestamp."""
    # Use binary search to find the index of the closest gps message
    idx = np.searchsorted(gps_msgs['timestamps'], timestamp)
    if idx == 0:
        return gps_msgs['data'][0]
    elif idx == len(gps_msgs['data']):
        return gps_msgs['data'][-1]
    else:
        # Choose the closest of the two neighboring messages
        prev_delta = abs(gps_msgs['timestamps'][idx-1] - timestamp)
        next_delta = abs(gps_msgs['timestamps'][idx] - timestamp)
        if prev_delta < next_delta:
            return gps_msgs['data'][idx-1]
        else:
            return gps_msgs['data'][idx]


def save_data_to_files(data, bag_timestamp, save_root, rosbag_name):
    """Save the extracted data to the files."""
    # Save the data as a numpy array
    npy_filename = f'{bag_timestamp}.npy'
    points_path = join(save_root, rosbag_name, "points")
    if not exists(points_path):
        makedirs(points_path)
    np.save(join(points_path, npy_filename), data)


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--points_topic', default="/radar_enhanced_pcl",
                        help='The topic containing the point cloud data')
    parser.add_argument('--gps_topic', default="/ublox/fix",
                        help='The topic containing the GPS data')
    parser.add_argument('--bags', nargs='+', required=True,
                        help='The list of bag files to process')
    parser.add_argument('--save_folder', default="ros_datasets/extracted_data",
                        help='Root folder to save extracted data')
    args = parser.parse_args()

    # List of bag files to process
    bag_filenames = sorted(
        [file_name for pattern in args.bags for file_name in glob.glob(
            pattern)]
    )

    for bag_filename in bag_filenames:
        rosbag_name = bag_filename.split("/")[-1].split("_")[0]
        print(f'Processing { bag_filename}...')

        # Lists to store the extracted data
        pcl_data = []
        gps_data = []

        # Open the bag file and extract the messages
        bag = rosbag.Bag(bag_filename)
        for topic, msg, t in tqdm(bag.read_messages(), desc=f"==> Reading ROS bag", leave=True, position=0):
            if topic == args.points_topic:
                points = np.array([[point.x, point.y, point.z]
                                  for point in msg.points])
                speed = np.array(msg.channels[0].values)
                power = np.array(msg.channels[2].values)

                pcl_data.append((t.to_sec(), np.hstack(
                    (points, speed.reshape(-1, 1), power.reshape(-1, 1)))))

            elif topic == args.gps_topic:
                gps_data.append(
                    (t.to_sec(), np.array([msg.latitude, msg.longitude, msg.altitude])))

        bag.close()

        # Convert lists to numpy arrays for faster processing
        pcl_data = np.array(
            pcl_data, dtype=[('timestamps', float), ('data', object)], ndmin=1)
        gps_data = np.array(
            gps_data, dtype=[('timestamps', float), ('data', object)], ndmin=1)

        # Sort the arrays by timestamp for faster searching
        pcl_data = np.sort(pcl_data, order='timestamps')
        gps_data = np.sort(gps_data, order='timestamps')

        gps_df = pd.DataFrame(
            columns=['timestamp', 'latitude', 'longitude', 'altitude'])

        # Extract the data from each PCL msg and save it to a numpy array
        for timestamp, pcl_msg in tqdm(pcl_data, desc=f"==> Writing data", leave=True, position=0):
            gps_info = extract_gps_msg(gps_data, timestamp)
            timestamp = "{:.6f}".format(timestamp).replace(".", "")
            new_row = {"timestamp": timestamp,
                       "latitude": gps_info[0], "longitude": gps_info[1], "altitude": gps_info[2]}

            gps_df.loc[len(gps_df)] = new_row

            save_data_to_files(
                pcl_msg, f'{timestamp}', args.save_folder, rosbag_name)
        csv_filename = join(args.save_folder, rosbag_name,
                            f"{rosbag_name}_gps.csv")
        if isfile(csv_filename):
            gps_df.to_csv(csv_filename, mode='a', header=False, index=False)
        else:
            gps_df.to_csv(csv_filename, header=True, index=False)
