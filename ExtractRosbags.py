#!/usr/bin/env python3

import rosbag
import math
import numpy as np
import pandas as pd
import argparse
from os.path import join, exists
from os import makedirs
import glob
from tqdm import tqdm
import cv2


def find_topics(topics, data_types):
    """Find topics that could potentially contain data of the given types."""
    possible_topics = []
    for data_type in data_types:
        possible_topics.extend(
            [topic for topic, (topic_type, _, _, _) in topics.items() if data_type in topic_type])
    return possible_topics


def user_select_topic(possible_topics, data_description):
    """Prompt user to select a topic from a list of possible topics."""
    if not possible_topics:
        raise ValueError(
            f"No topics found for {data_description}. This data type is essential.")
    print(f"Select a topic for {data_description}:")
    possible_topics.append(None)
    for idx, topic in enumerate(possible_topics):
        print(f"{idx + 1}: {topic}")
    selection = int(input("Enter the number of your choice: ")) - 1
    return possible_topics[selection]


def get_topics_to_read(selected_topics, bag):
    """
    Validate and select topics for data extraction.
    If topics in selected_topics are not valid for the current bag file, user is prompted to select new topics.

    Args:
    selected_topics (dict): A dictionary with keys 'points_topic', 'gps_topic', 'image_topic' and their current values.
    bag (rosbag.Bag): The currently opened rosbag file.

    """
    topics = bag.get_type_and_topic_info()[1]
    current_topics = topics.keys()
    topics_to_read = []

    # GPS topic validation and selection
    if selected_topics['gps']['topic'] is None or selected_topics['gps']['topic'] not in current_topics:
        possible_gps_topics = find_topics(topics, ['sensor_msgs/NavSatFix'])
        assert possible_gps_topics, "Can't find any GPS topic."
        selected_topics['gps']['topic'] = user_select_topic(
            possible_gps_topics, 'GPS') if len(possible_gps_topics) > 1 else possible_gps_topics[0]
    topics_to_read.append(selected_topics['gps']['topic'])

    # Points topic validation and selection
    if selected_topics['points']['topic'] is None or selected_topics['points']['topic'] not in current_topics:
        possible_points_topics = find_topics(
            topics, ['sensor_msgs/PointCloud2', 'msgs_radar/RadarScanExtended'])
        selected_topics['points']['topic'] = user_select_topic(
            possible_points_topics, 'Points') if possible_points_topics else None
    if selected_topics['points']['topic']:
        topics_to_read.append(selected_topics['points']['topic'])

    # Image topic validation and selection
    if selected_topics['image']['topic'] is None or selected_topics['image']['topic'] not in current_topics:
        possible_image_topics = find_topics(
            topics, ['sensor_msgs/Image', 'sensor_msgs/CompressedImage'])
        selected_topics['image']['topic'] = user_select_topic(
            possible_image_topics, 'Image') if possible_image_topics else None
    selected_topics['image']['is_compressed'] = selected_topics['image']['topic'] in find_topics(topics, ['sensor_msgs/CompressedImage'])
    if selected_topics['image']['topic']:
        topics_to_read.append(selected_topics['image']['topic'])

    return selected_topics, topics_to_read


def extract_msg(msgs, timestamp):
    # Use binary search to find the index of the closest gps message
    idx = np.searchsorted(msgs['timestamps'], timestamp)
    if idx == 0:
        return msgs['data'][0]
    elif idx == len(msgs['data']):
        return msgs['data'][-1]
    else:
        # Choose the closest of the two neighboring messages
        prev_delta = abs(msgs['timestamps'][idx-1] - timestamp)
        next_delta = abs(msgs['timestamps'][idx] - timestamp)
        if prev_delta < next_delta:
            return msgs['data'][idx-1]
        else:
            return msgs['data'][idx]


def save_image(image_msg, frame_time, save_root, rosbag_name, is_compressed):
    """Save the image data to a file."""
    image_path = join(save_root, rosbag_name, "images")
    if not exists(image_path):
        makedirs(image_path)

    if is_compressed:
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    else:  # Assume raw image
        image_np = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(
            image_msg.height, image_msg.width, -1)
        image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)


    # You can choose other formats like jpg
    image_filename = f'{frame_time}.png'
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
    parser.add_argument('--points_topic', default=None,
                        help='The topic containing the point cloud data')
    parser.add_argument('--gps_topic', default=None,
                        help='The topic containing the GPS data')
    parser.add_argument('--image_topic', default=None,
                        help='The topic containing the image data')
    parser.add_argument('--bags', nargs='+', required=True,
                        help='The list of bag files to process')
    parser.add_argument('--save_folder', default="ros_datasets/extracted_data",
                        help='Root folder to save extracted data')
    parser.add_argument('--fps', type=float, default=4, help='Sampling interval for GPS data in seconds')


    args = parser.parse_args()


    selected_topics = {
        'points':{
            'topic': args.points_topic,
            'is_polar': False
            },
        'gps': {
            'topic': args.gps_topic
        },
        'image': 
        {
            'topic': args.image_topic,
            'is_compressed': False
        }
    }

    # List of bag files to process
    bag_filenames = sorted(
        [file_name for pattern in args.bags for file_name in glob.glob(
            pattern)]
    )
    ds_dict = {}

    for bag_filename in bag_filenames:
        filename = bag_filename.split("/")[-1]
        if "_" in filename:
            rosbag_name = "_".join(filename.split("_")[:-2])
            date = "".join(filename.split("_")[-2].split("-")[:3])
            rosbag_name = f"{date}_{rosbag_name}"
        else:
            rosbag_name = filename.split(".")[0]
        if rosbag_name not in ds_dict:
            ds_dict[rosbag_name] = [bag_filename]
        else:
            ds_dict[rosbag_name].append(bag_filename)

    for rosbag_name, bags_filename in ds_dict.items():
        points_data = []
        gps_data = []
        image_data = []
        for indx, bag_filename in enumerate(bags_filename, 1):
            print(f'Processing {rosbag_name}: {indx}/{len(bags_filename)}')


            bag = rosbag.Bag(bag_filename, "r")

            if indx == 1:
                selected_topics, topics_to_read = get_topics_to_read(
                    selected_topics, bag)

                if selected_topics['points']['topic'] == "/radar_scan" and selected_topics['gps']['topic'] == "/gps/fix":
                    selected_topics["points"]["is_polar"] = True

            total_messages = bag.get_message_count(
                topic_filters=topics_to_read)


            with tqdm(total=total_messages, desc=f"Extracting from {bag_filename}", unit="msg") as pbar:
                for topic, msg, t in bag.read_messages(topics=topics_to_read):
                    if topic == selected_topics['points']['topic']:
                        if selected_topics['points']['is_polar']:
                            points = np.array([[target.range * math.cos(target.elevation) * math.cos(target.azimuth),
                                                target.range *
                                                math.cos(target.elevation) *
                                                math.sin(target.azimuth),
                                                target.range * math.sin(target.elevation)]
                                               for target in msg.targets])
                            speed = np.array(
                                [target.velocity for target in msg.targets])
                            power = np.array(
                                [target.power for target in msg.targets])
                        else:
                            points = np.array([[point.x, point.y, point.z]
                                               for point in msg.points])
                            speed = np.array(msg.channels[0].values)
                            power = np.array(msg.channels[2].values)

                        points_data.append((t.to_sec(), np.hstack(
                            (points, speed.reshape(-1, 1), power.reshape(-1, 1)))))

                    elif topic == selected_topics['gps']['topic']:
                        gps_data.append(
                            (t.to_sec(), np.array([msg.latitude, msg.longitude, msg.altitude])))
                    elif topic == selected_topics['image']['topic']:
                        image_data.append((t.to_sec(), msg))
                    pbar.update(1)

            bag.close()


        assert len(gps_data) != 0, "No GPS data extracted."

        # Convert lists to numpy arrays for faster processing
        pcl_data = np.array(
            points_data, dtype=[('timestamps', float), ('data', object)], ndmin=1)
        gps_data = np.array(
            gps_data, dtype=[('timestamps', float), ('data', object)], ndmin=1)
        img_data = np.array(
            image_data, dtype=[('timestamps', float), ('data', object)], ndmin=1
        )

        gps_data = np.sort(gps_data, order='timestamps')

        # Sort the arrays by timestamp for faster searching
        pcl_data = np.sort(pcl_data, order='timestamps')
        img_data = np.sort(img_data, order='timestamps')


        csv_filename = join(args.save_folder, rosbag_name,
                            f"{rosbag_name}_gps.csv")

        gps_df = pd.DataFrame(
            columns=['timestamp', 'latitude', 'longitude', 'altitude'])

        interval = 1.0 / args.fps
        last_processed_time = None
        # Extract the data from each PCL msg and save it to a numpy array
        for timestamp, gps_info in tqdm(gps_data, desc=f"==> Writing data", leave=True, position=0):
            if last_processed_time is not None and (timestamp - last_processed_time) < interval:
                continue
            last_processed_time = timestamp
            if len(pcl_data) > 0:
                pcl_msg = extract_msg(pcl_data, timestamp)
                save_data_to_files(
                    pcl_msg, f'{timestamp}', args.save_folder, rosbag_name)
            if len(img_data) > 0:
                img_msg = extract_msg(img_data, timestamp)
                save_image(img_msg, "{:.6f}".format(timestamp).replace(
                    ".", ""), args.save_folder, rosbag_name, selected_topics["image"]["is_compressed"])
            timestamp = "{:.6f}".format(timestamp).replace(".", "")
            new_row = {"timestamp": timestamp,
                       "latitude": gps_info[0], "longitude": gps_info[1], "altitude": gps_info[2]}

            gps_df.loc[len(gps_df)] = new_row


        # Write updated DataFrame to CSV
        gps_df.to_csv(csv_filename, header=True, index=False)
