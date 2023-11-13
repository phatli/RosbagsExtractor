#!/usr/bin/env python3

import rosbag
import math
import numpy as np
import pandas as pd
import argparse
from os.path import join, isfile, exists
from os import makedirs
import glob
from tqdm import tqdm
import cv2



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

def extract_img_msg(img_msgs, timestamp):
    idx = np.searchsorted(img_msgs['timestamps'], timestamp)
    if idx == 0:
        return img_msgs['data'][0]
    elif idx == len(img_msgs['data']):
        return img_msgs['data'][-1]
    else:
        prev_delta = abs(img_msgs['timestamps'][idx-1] - timestamp)
        next_delta = abs(img_msgs['timestamps'][idx] - timestamp)
        if prev_delta < next_delta:
            return img_msgs['data'][idx-1]
        else:
            return img_msgs['data'][idx]

def save_image(image_msg, frame_time, save_root, rosbag_name, is_compressed):
    """Save the image data to a file."""
    image_path = join(save_root, rosbag_name, "images")
    if not exists(image_path):
        makedirs(image_path)
    
    if is_compressed:
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    else:  # Assume raw image
        image_np = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)

    image_filename = f'{frame_time}.png'  # You can choose other formats like jpg
    cv2.imwrite(join(image_path, image_filename), image_np)

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
    parser.add_argument('--image_topic', default=None,
                        help='The topic containing the image data')
    parser.add_argument('--bags', nargs='+', required=True,
                        help='The list of bag files to process')
    parser.add_argument('--save_folder', default="ros_datasets/extracted_data",
                        help='Root folder to save extracted data')
    args = parser.parse_args()

    is_polar_coord = False
    if args.points_topic == "/radar_scan" and args.gps_topic == "/gps/fix":
        is_polar_coord = True

    # List of bag files to process
    bag_filenames = sorted(
        [file_name for pattern in args.bags for file_name in glob.glob(
            pattern)]
    )

    for bag_filename in bag_filenames:
        filename = bag_filename.split("/")[-1]
        if "_" in filename:
            rosbag_name = "_".join(filename.split("_")[:-2])
            date = "".join(filename.split("_")[-2].split("-")[:3])
            rosbag_name = f"{date}_{rosbag_name}"
        else:
            rosbag_name = filename.split(".")[0]
        print(f'Processing { bag_filename}...')

        # Lists to store the extracted data
        pcl_data = []
        gps_data = []
        topics_to_read = [args.points_topic, args.gps_topic]
        if args.image_topic:
            topics_to_read.append(args.image_topic)
            img_timestamps = []
            is_compressed = None
        # Open the bag file and extract the messages
        bag = rosbag.Bag(bag_filename, "r")
        total_messages = bag.get_message_count(topic_filters=topics_to_read)
        with tqdm(total=total_messages, desc=f"Extracting from {bag_filename}", unit="msg") as pbar:
            for topic, msg, t in bag.read_messages(topics=topics_to_read):
                if topic == args.points_topic:
                    if is_polar_coord:
                        points = np.array([[target.range * math.cos(target.elevation) * math.cos(target.azimuth),
                                            target.range *
                                            math.cos(target.elevation) *
                                            math.sin(target.azimuth),
                                            target.range * math.sin(target.elevation)]
                                        for target in msg.targets])
                        speed = np.array(
                            [target.velocity for target in msg.targets])
                        power = np.array([target.power for target in msg.targets])
                    else:
                        points = np.array([[point.x, point.y, point.z]
                                        for point in msg.points])
                        speed = np.array(msg.channels[0].values)
                        power = np.array(msg.channels[2].values)

                    pcl_data.append((t.to_sec(), np.hstack(
                        (points, speed.reshape(-1, 1), power.reshape(-1, 1)))))

                elif topic == args.gps_topic:
                    gps_data.append(
                        (t.to_sec(), np.array([msg.latitude, msg.longitude, msg.altitude])))
                elif topic == args.image_topic:
                    img_timestamps.append((t.to_sec(), msg))
                    if is_compressed is None:
                        is_compressed = msg._type == "sensor_msgs/CompressedImage"
                pbar.update(1)


        bag.close()

        assert len(pcl_data) !=0 and len(gps_data) != 0, "No data extracted from the bag file."
        if args.image_topic:
            assert is_compressed is not None, "Cannot find image topic in rosbag."

        # Convert lists to numpy arrays for faster processing
        pcl_data = np.array(
            pcl_data, dtype=[('timestamps', float), ('data', object)], ndmin=1)
        gps_data = np.array(
            gps_data, dtype=[('timestamps', float), ('data', object)], ndmin=1)
        if args.image_topic:
            img_data = np.array(
                img_timestamps, dtype=[('timestamps', float), ('data', object)], ndmin=1
            )

        # Sort the arrays by timestamp for faster searching
        pcl_data = np.sort(pcl_data, order='timestamps')
        gps_data = np.sort(gps_data, order='timestamps')
        if args.image_topic:
            img_data = np.sort(img_data, order='timestamps')

        gps_df = pd.DataFrame(
            columns=['timestamp', 'latitude', 'longitude', 'altitude'])

        # Extract the data from each PCL msg and save it to a numpy array
        for timestamp, pcl_msg in tqdm(pcl_data, desc=f"==> Writing data", leave=True, position=0):
            gps_info = extract_gps_msg(gps_data, timestamp)
            if args.image_topic:
                img_msg = extract_img_msg(img_data, timestamp)
                save_image(img_msg, "{:.6f}".format(timestamp).replace(".", ""), args.save_folder, rosbag_name, is_compressed)
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
