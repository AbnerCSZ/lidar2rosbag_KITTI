
How to build with catkin:

```
$ cd ~/catkin_ws/src/
$ git clone  git@github.com:AbnerCSZ/lidar2rosbag_KITTI.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```

Running:

```
rosrun lidar2rosbag lidar2rosbag KITTI_input_dir output_name
```

Please confirm that the input path contains the file `times.txt` and the folder `velodyne`.

```
└── dataset
    └── sequences
        └── 04
            ├── calib.txt
            ├── image_0 [271 entries exceeds filelimit, not opening dir]
            ├── image_1 [271 entries exceeds filelimit, not opening dir]
            ├── times.txt
            └── velodyne [271 entries exceeds filelimit, not opening dir]
```

For example:
```
rosrun lidar2rosbag lidar2rosbag /data/KITTI/dataset/sequences/04/ 04
```
or
```
rosrun lidar2rosbag lidar2rosbag /data/KITTI/dataset/sequences/01/ bag01
```
and so on...

