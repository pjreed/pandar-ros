ROS package supporting Hesai Pandar40 LIDAR
---
This package runs as an ROS node. The data from Pandar40 LIDAR can be either grabbed from the ethernet, or read from a pcap file.  
**Note**:  Source code in `pandar grabber` could be used as a component in any C++ project to interface with Pandar40.

## Prerequisite
1. libpcap-dev  
  On Ubuntu, `sudo apt-get install libpcap-dev`
2. pcl >= 1.7  

## Compile
```
catkin_make
```

## Run
- live data from ethernet  
```
roslaunch pandar_ros pandar.launch device_ip:=<device ip> port:=<port> calib:=<path to .csv calibration file>
```
- read data from pcap file  
```
roslaunch pandar_ros pandar.launch pcap:=<full path to pcap file> calib:=<path to .csv calibration file>
```

### Message Published 
* /pandar\_node/pandar\_points [sensor\_msgs/PointCloud2]

## Tested environment
* ROS indigo on Ubuntu14.04
* ROS kinetic on Ubuntu16.04
* ROS jade on Ubuntu14.04

## [Download sample data](http://www.hesaitech.com/un-file.html?backUrl=/autonomous_driving.html)


基于pandar grabber的ROS包
---
这个包在ROS中创建一个节点，用于从以太网中读取禾赛Pandar 40线激光雷达的数据，亦可用于从pcap文件中读取离线保存的雷达数据。
其中**pandar grabber**路径下的程序可以单独应用于其他C++程序。

## 依赖
1. libpcap-dev  
  On Ubuntu, `sudo apt-get install libpcap-dev`
2. pcl >= 1.7  

## 编译
```
catkin_make
```

## 运行
从以太网中读取实时雷达数据
```
roslaunch pandar_ros pandar.launch device_ip:=<device ip> port:=<port> calib:=<path to .csv calibration file>
```
从pcap文件中读取雷达数据
```
roslaunch pandar_ros pandar.launch pcap:=<full path to pcap file> calib:=<path to .csv calibration file>
```

### 发布的消息 
* /pandar\_node/pandar\_points [sensor\_msgs/PointCloud2]

## 测试过的环境
* ROS indigo on Ubuntu14.04
* ROS kinetic on Ubuntu16.04
* ROS jade on Ubuntu14.04

## [数据下载地址](http://www.hesaitech.com/un-file.html?backUrl=/autonomous_driving.html)
