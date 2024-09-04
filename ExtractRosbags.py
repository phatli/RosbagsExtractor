#!/usr/bin/env python3

import rosbag
import math
import numpy as np
import ros_numpy
import pandas as pd
import argparse
from os.path import join, exists
from os import makedirs
import glob
from tqdm import tqdm
import cv2
import re


class SensorData():
    def __init__(self, topic, msgtype, save_folder, rosbag_name):
        self.topic = topic
        self.msgtype = msgtype
        self.data_lst = []
        self.data_np = None
        self.save_folder = save_folder
        self.rosbag_name = rosbag_name

    def sort_data(self):
        data = np.array(self.data_lst, dtype=[
                        ('timestamps', float), ('data', object)], ndmin=1)
        self.data_np = np.sort(data, order='timestamps')

    def __len__(self):
        return len(self.data_lst)


class RadarData(SensorData):
    def __init__(self, args):
        super().__init__(*args)
        # hardcoded for now
        if self.topic == "/radar_scan":
            self.is_polar = True
        else:
            self.is_polar = False

    def extract_data(self, msg, t):
        """
        Extract radar data from the message and append to the data list.
            msg: radar message
            t: timestamp of the message
        """
        if self.is_polar:
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
        self.data_lst.append((t.to_sec(), np.hstack(
            (points, speed.reshape(-1, 1), power.reshape(-1, 1)))))

    def save_data(self, timestamp):
        if self.data_np is None:
            self.sort_data()
        if len(self.data_np) > 0:
            pcl_msg = extract_msg(self.data_np, timestamp)
            save_npdata_to_files(
                pcl_msg, "{:.6f}".format(timestamp).replace(
                ".", ""), self.save_folder, self.rosbag_name, self.topic)


class LidarData(SensorData):
    def __init__(self, args):
        super().__init__(*args)

    def extract_data(self, msg, t):
        points = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        self.data_lst.append((t.to_sec(), points))

    def save_data(self, timestamp):
        if self.data_np is None:
            self.sort_data()
        if len(self.data_np) > 0:
            pcl_msg = extract_msg(self.data_np, timestamp)
            save_npdata_to_files(
                pcl_msg, "{:.6f}".format(timestamp).replace(
                ".", ""), self.save_folder, self.rosbag_name, self.topic)


class ImageData(SensorData):
    def __init__(self, args):
        super().__init__(*args)
        self.is_compressed = self.msgtype == 'sensor_msgs/CompressedImage'

    def extract_data(self, msg, t):
        self.data_lst.append((t.to_sec(), msg))

    def save_data(self, timestamp):
        if self.data_np is None:
            self.sort_data()
        if len(self.data_np) > 0:
            img_msg = extract_msg(self.data_np, timestamp)
            save_image(img_msg, "{:.6f}".format(timestamp).replace(
                ".", ""), self.save_folder, self.rosbag_name, self.is_compressed, self.topic)


class FixData(SensorData):
    def __init__(self, args):
        super().__init__(*args)

    def set_tc(self, tc):
        self.tc = tc

    def extract_data(self, msg, t):
        self.data_lst.append(
            (t.to_sec() - self.tc, np.array([msg.latitude, msg.longitude, msg.altitude])))


msgtypes_dataclss = {
    'sensor_msgs/NavSatFix': FixData,
    'sensor_msgs/PointCloud2': LidarData,
    'msgs_radar/RadarScanExtended': RadarData,
    'sensor_msgs/Image': ImageData,
    'sensor_msgs/CompressedImage': ImageData,
    'sensor_msgs/PointCloud': RadarData
}


def multi_select_prompt(options):
    """Prompts the user to select multiple options from a list."""
    selected_indices = []
    while True:
        # Display available options with indices
        for index, option in enumerate(options):
            print(
                f"{index + 1}: {option}{' (selected)' if index in selected_indices else ''}")

        # Get user input
        input_indices = input(
            "Select options by entering indices separated by space (e.g., 1 2 3) or type 'done' to finish: ")

        # Handle 'done' input to move to confirmation
        if input_indices.lower() == 'done':
            if not selected_indices:
                print("No options selected. Please select at least one option.")
                continue
            break

        # Parse indices and update selected options
        try:
            # Convert input string to list of indices, adjust for zero-based indexing
            input_indices = [int(i) - 1 for i in input_indices.split()]

            # Toggle selection based on input indices
            for index in input_indices:
                if index in selected_indices:
                    selected_indices.remove(index)
                elif index >= 0 and index < len(options):
                    selected_indices.append(index)
                else:
                    print(
                        f"Index {index + 1} is out of range. Please select a valid index.")
        except ValueError:
            print("Invalid input. Please enter only numerical indices.")
            continue

    # Show final selections and confirm
    final_selections = [options[i] for i in selected_indices]
    return final_selections



def ask_time_compensation():
    """Prompt the user to determine the specific time compensation needed for GPS data."""
    print("Do you need to compensate the GPS timestamps to match UTC time?")
    print("1: Yes, I will specify the compensation.")
    print("2: No compensation needed.")
    print("3: Use standard GPS-UTC offset (18 seconds).")
    choice = input("Enter your choice (1, 2, or 3): ")

    if choice == '1':
        compensation = input(
            "Enter the time compensation in seconds (positive or negative): ")
        try:
            return float(compensation)
        except ValueError:
            print("Invalid input. Please enter a numeric value.")
            return ask_time_compensation()  # Recursively prompt if invalid
    elif choice == '2':
        return 0
    elif choice == '3':
        return 18
    else:
        print("Invalid input. Please enter 1, 2, or 3.")
        return ask_time_compensation()


def get_rosbag_name(filename):
    # Remove the ".bag" suffix and split the filename to remove the index
    parts = filename[:-4].split('_')

    # Remove the last part (index) and rejoin the remaining parts
    remaining = '_'.join(parts[:-1])

    # Regular expression to match the date format
    date_pattern = re.compile(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}')

    # Search for the date and format it
    match = date_pattern.search(remaining)
    if match:
        date = match.group(0)
        formatted_date = date[:10].replace('-', '')
        # Remove the date from the string
        name = date_pattern.sub('', remaining).strip('_')
    else:
        formatted_date = None
        name = remaining  # Use the whole remaining string as name if no date found

    # Combine the formatted date and name
    combined_name = f"{formatted_date}_{name}" if formatted_date else name

    return combined_name


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


def get_topics_to_read(selected_topics, bag, save_folder, rosbag_name):
    """
    Validate and select topics for data extraction.
    If topics in selected_topics are not valid for the current bag file, user is prompted to select new topics.

    Args:
    selected_topics (dict)
    bag (rosbag.Bag)

    """
    topics = bag.get_type_and_topic_info()[1]

    filtered_topics_msgtypes = {topic: topic_type for topic, (topic_type, _, _, _) in topics.items(
    ) if topic_type in msgtypes_dataclss}

    if selected_topics['base'] is None:
        fix_topics = find_topics(topics, ['sensor_msgs/NavSatFix'])
        base_topic = user_select_topic(fix_topics, 'GPS') if len(
            fix_topics) > 1 else fix_topics[0]
        selected_topics['base'] = base_topic
    else:
        assert selected_topics[
            'base'] in filtered_topics_msgtypes, f"Invalid topic: {selected_topics['base']}"
        base_topic = selected_topics['base']

    if len(selected_topics['sensors']) == 0:
        possible_sensors_topics = list(filtered_topics_msgtypes.keys())
        possible_sensors_topics.remove(base_topic)
        selected_topic_names = multi_select_prompt(possible_sensors_topics)
        selected_topics['sensors'] = selected_topic_names
    else:
        for topic in selected_topics['sensors']:
            assert topic in filtered_topics_msgtypes, f"Invalid topic: {topic}"
        selected_topic_names = selected_topics['sensors']

    selected_topics['data'] = {topic: msgtypes_dataclss[filtered_topics_msgtypes[topic]](
        [topic, filtered_topics_msgtypes[topic], save_folder, rosbag_name]) for topic in [base_topic, *selected_topic_names]}

    
    return selected_topics


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


def save_image(image_msg, frame_time, save_root, rosbag_name, is_compressed, topic):
    """Save the image data to a file."""
    image_path = join(save_root, rosbag_name,
                      topic.replace("/", "_").strip("_"))
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


def save_npdata_to_files(data, bag_timestamp, save_root, rosbag_name, topic):
    """Save the extracted data to the files."""
    # Save the data as a numpy array
    npy_filename = f'{bag_timestamp}.npy'
    points_path = join(save_root, rosbag_name,
                       topic.replace("/", "_").strip("_"))
    if not exists(points_path):
        makedirs(points_path)
    np.save(join(points_path, npy_filename), data)


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--base_topic', default=None,
                        help='Base topic for other topics to sync, usually gps topic')
    parser.add_argument('--topics', nargs='+', default=[],
                        help='A list of topics to extract data from')
    parser.add_argument('--bags', nargs='+', required=True,
                        help='The list of bag files to process')
    parser.add_argument('--save_folder', default="ros_datasets/extracted_data",
                        help='Root folder to save extracted data')
    parser.add_argument('--fps', type=float, default=4,
                        help='Sampling interval for GPS data in seconds')
    parser.add_argument('--gps_tc', type=float, default=0,
                        help='Time compensation between GPS and other sensors')

    args = parser.parse_args()

    selected_topics = {"base": args.base_topic}
    selected_topics['sensors'] = args.topics

    # List of bag files to process
    bag_filenames = sorted(
        [file_name for pattern in args.bags for file_name in glob.glob(
            pattern)]
    )
    ds_dict = {}

    for bag_filename in bag_filenames:
        filename = bag_filename.split("/")[-1]
        if "_" in filename:
            rosbag_name = get_rosbag_name(filename)
        else:
            rosbag_name = filename.split(".")[0]
        if rosbag_name not in ds_dict:
            ds_dict[rosbag_name] = [bag_filename]
        else:
            ds_dict[rosbag_name].append(bag_filename)

    for rosbag_name, bags_filename in ds_dict.items():
        for indx, bag_filename in enumerate(bags_filename, 1):
            print(f'Processing {rosbag_name}: {indx}/{len(bags_filename)}')

            bag = rosbag.Bag(bag_filename, "r")

            if indx == 1:
                selected_topics = get_topics_to_read(
                    selected_topics, bag, args.save_folder, rosbag_name)
                topics_to_read = [selected_topics['base']] + selected_topics['sensors']
                base_topic = selected_topics['base']

                if selected_topics['data'][base_topic].msgtype == 'sensor_msgs/NavSatFix':
                    selected_topics['data'][base_topic].set_tc(args.gps_tc)

            total_messages = bag.get_message_count(
                topic_filters=topics_to_read)

            with tqdm(total=total_messages, desc=f"Extracting from {bag_filename}", unit="msg") as pbar:
                for topic, msg, t in bag.read_messages(topics=topics_to_read):
                    selected_topics['data'][topic].extract_data(msg, t)
                    pbar.update(1)
            bag.close()

        assert len(selected_topics['data'][base_topic]) != 0, "No GPS data extracted."

        # Convert lists to numpy arrays for faster processing
        selected_topics['data'][base_topic].sort_data()

        if selected_topics['data'][base_topic].msgtype == 'sensor_msgs/NavSatFix' and selected_topics['data'][base_topic].tc != 0:
            rosbag_name = f"{rosbag_name}-tc{selected_topics['gps']['tc']}"

        csv_filename = join(args.save_folder, rosbag_name,
                            f"{rosbag_name}_gps.csv")

        gps_df = pd.DataFrame(
            columns=['timestamp', 'latitude', 'longitude', 'altitude'])
        gps_data = selected_topics['data'][base_topic].data_np

        interval = 1.0 / args.fps
        last_processed_time = None
        # Extract the data from each PCL msg and save it to a numpy array
        for timestamp, gps_info in tqdm(gps_data, desc=f"==> Writing data", leave=True, position=0):
            if last_processed_time is not None and (timestamp - last_processed_time) < interval:
                continue
            last_processed_time = timestamp
            for topic in selected_topics['sensors']:
                selected_topics['data'][topic].save_data(timestamp)
            timestamp = "{:.6f}".format(timestamp).replace(".", "")
            new_row = {"timestamp": timestamp,
                       "latitude": gps_info[0], "longitude": gps_info[1], "altitude": gps_info[2]}

            gps_df.loc[len(gps_df)] = new_row

        # Write updated DataFrame to CSV
        gps_df.to_csv(csv_filename, header=True, index=False)
