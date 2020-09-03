# LiDAR Visualization of KITTI Data 
Inspired by [Alex Staravoitau](github.com/navoshta)

This script visualizes the provided LiDAR Data in the raw [KITTI](http://www.cvlibs.net/datasets/kitti/raw_data.php) datasets. 

## Getting Started 
You need all of this to run the script 
### Dependencies
- Python 3.6
- Numpy
- Matplotlib
- glob 
- pykitti 


Because this script uses [pykitti](https://github.com/utiasSTARS/pykitti), it is assumed that the raw datasets is already downloaded to your local machine. The file tree should look like the following after unzipping the synced+rectified data, calibration, and tracklets files: 
```
path/to/directory
│   
└───2011_09_26
|   |   calib_cam_to_cam.txt
|   |   calib_imu_to_velo.txt
|   |   calib_velo_to_cam.txt
|   |
│   └───2011_09_26_drive_0001_sync
|   |   |   tracklet_labels.xml
|   |   |
│   |   └───image_00
|   |   |   |   data 
|   |   |   |   timestamps.txt
│   |   └───image_01
|   |   |   |   data
|   |   |   |   timestamps.txt
│   |   └───image_02
│   |   └───image_03
|   |   └───oxts
|   |   |   |   data
|   |   |   |   dataformat.txt
|   |   |   |   timestamps.txt
|   |   └───velodyne_points
|   |   |   |  data
|   |   |   |  timestamps.txt 
|   |   |   |  timestamps_end.txt
|   |   |   |  timestamps_start.txt
|   |
|   └───2011_09_26_drive_0002_sync
|   |   |   ...
```

Apart from having the raw dataset downloaded, the project structure should look similar to the following: 
| File                   | Description                                                                                      |
| ---------------------- | ------------------------------------------------------------------------------------------------ |
| `velodyne.py`  | Python script with dataset visualisation routines and output.                                 |
| `parseTrackletXML.py`  | Methods for parsing tracklets (e.g. dataset labels), originally created by Christian Herdtweck.  |
                                                         
## Usage 
There are a few command-line arguments that are *required* to run the script 

| Argument               | Description                                                                                      |
| ---------------------- | ------------------------------------------------------------------------------------------------ |
| `--basedir`            | Path to the directory storing the raw datasets.                                                  |
| `--drive`              | This is the drive number indicated by the raw file ex:0001, 0002, 000 etc.                       |
| `--date`               | Date of the drive indicated by the file.                                                         |
| `--xmlFile`            | Path to tracklets xml file                                                                       |

```python
python velodyne.py --basedir path/to/basedir --drive 0001 --date 2011_09_26 --xmlFile 2011_09_26/2011_09_26_drive_0001_sync/tracklet_labels.xml 
```
