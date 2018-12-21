
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
rosrun lidar2rosbag lidar2rosbag /data/KITTI/dataset/sequences/04/ 04
```

