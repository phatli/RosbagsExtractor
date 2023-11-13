# Rosbag PCL Extractor

This script is used to extract point cloud data from ROSBAG files and save them as numpy arrays. It also extracts GPS data and saves it as a CSV file, with each row containing the timestamp, latitude, longitude, and altitude.

## Usage

`python rosbag_pcl_extractor.py --points_topic <PCL_TOPIC> --gps_topic <GPS_TOPIC> --image_topic <IMG_TOPIC> --bags <BAG_FILENAME1> <BAG_FILENAME2> ... --save_folder <SAVE_FOLDER>`

#### Arguments:

- `--points_topic`: The ROS topic containing the point cloud data (default: `/radar_enhanced_pcl`).
- `--gps_topic`: The ROS topic containing the GPS data (default: `/ublox/fix`).
- `--image_topic`: The ROS topic containing the image data (default: None)
- `--bags`: A list of ROSBAG files to process.
- `--save_folder`: The root folder to save the extracted data (default: `../extracted_data`).

## Output

- Point cloud data is saved as numpy arrays in the `points` subfolder of the `SAVE_FOLDER`, with the filename as the timestamp of the message.
- Image data is saved as `.png` in the `images` subfolder of the `SAVE_FOLDER`, with the filename as the timestamp of the message.
- GPS data is saved as a CSV file in the `SAVE_FOLDER`, with the filename as `<ROSBAG_NAME>_gps.csv`. If the file already exists, new data is appended to it.