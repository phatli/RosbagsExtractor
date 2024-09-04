# ROS Bag Data Extractor

- [ROS Bag Data Extractor](#ros-bag-data-extractor)
  - [Features](#features)
  - [Requirements](#requirements)
  - [Setup](#setup)
  - [Usage](#usage)

This Python script is designed to process ROS bag files to extract GPS, point cloud, and image data, handling various topic types and allowing dynamic topic selection. It features an interactive mode to choose topics if not specified.

## Features

- **Dynamic Topic Selection**: If the topics are not specified or are not present in the bag, the script will prompt the user to choose from available topics.
- **Time Compensation**: Option to adjust GPS timestamps to match UTC, with custom compensation settings.
- **Data Extraction**: Extracts and processes GPS data, point cloud data, images and any msgtypes from specified topics within ROS bag files.
- **Data Saving**: Extracted data is saved to specified output directories, organized by dataset names and topic names. Dataset names are derived from rosbags name.

## Requirements

- Python 3
- ROS environment
- OpenCV (cv2), NumPy (numpy), Pandas (pandas), and tqdm libraries

## Setup

1. **Install Dependencies**: Ensure that you have ROS and the required Python packages installed. You can install the necessary Python packages using pip:
   pip install numpy pandas opencv-python tqdm

2. **Prepare ROS Environment**: This script assumes that you have a ROS environment set up and that you are familiar with the basics of ROS topics and bag files.

## Usage

1. **Prepare Your Bag Files**: Place your .bag files in a directory or specify the path to each bag file directly.

2. **Specify Topics (Optional)**: If known, you can specify the topics for GPS, point cloud, and image data via command-line arguments. If topics are not specified, the script will interactively prompt you to choose topics from each ROS bag file.

1. **Run the Script**:
   Use the following command to run the script, replacing <your_bag_files> with your bag file paths:

   ```shell
   python3 ExtractRosbags.py --bags <your_bag_files> --save_folder <output_directory> --topics <topic_1> <topic_2>
   ```

   Here are the available arguments:

   - --bags: List of bag files to process (required).
   - --save_folder: Root folder to save extracted data (default: ros_datasets/extracted_data).
   - --base_topic: Base topic for other topics to sync, usually gps topic. (optional, if not specified, use the gps topic)
   - --topics: List of sensors' topics to extract data from (optional, if not specified, would prompt later).
   - --fps: Frame rate to sample the GPS data (default: 4 frames per second).
   - --gps_tc: Time compensation in seconds to adjust GPS timestamps (optional).


