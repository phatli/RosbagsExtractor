# ROS Bag Data Extractor

- [ROS Bag Data Extractor](#ros-bag-data-extractor)
  - [Features](#features)
  - [Requirements](#requirements)
  - [Setup](#setup)
  - [Usage](#usage)
  - [Example](#example)


This Python script is designed to process ROS bag files to extract GPS, point cloud, and image data, handling various topic types and allowing dynamic topic selection. It features an interactive mode to choose topics if not specified, and it can handle both compressed and uncompressed image data.

## Features

- **Dynamic Topic Selection**: If the topics are not specified or are not present in the bag, the script will prompt the user to choose from available topics.
- **Data Extraction**: Extracts and processes GPS data, point cloud data, and images from specified topics within ROS bag files.
- **Data Saving**: Extracted data is saved to specified output directories, organized by dataset names derived from file names.
- **Compression Handling**: Determines if image data is compressed and processes it accordingly.

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

2. **Specify Topics (Optional)**: If known, you can specify the topics for GPS, point cloud, and image data via command-line arguments. If not specified, the script will allow you to select from available topics in each bag file.

3. **Run the Script**:
   Use the following command to run the script, replacing <your_bag_files> with your bag file paths:

   ```shell
   python3 extract_ros_data.py --bags <your_bag_files> --save_folder <output_directory>
   ```

   Here are the available arguments:

   - --points_topic: Topic for point cloud data.
   - --gps_topic: Topic for GPS data.
   - --image_topic: Topic for image data.
   - --bags: List of bag files to process (required).
   - --save_folder: Root folder to save extracted data (default: ros_datasets/extracted_data).
   - --fps: Frame rate to sample the GPS data (default: 4 frames per second).

## Example

To process files located in /path/to/bags and save the extracted data in /path/to/output, run:

```shell
python3 extract_ros_data.py --bags /path/to/bags/*.bag --save_folder /path/to/output
```

If topics are not specified, the script will interactively prompt you to choose topics from each ROS bag file. This process is repeated for each unique dataset identified by the bag filenames.

