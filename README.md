cg_mrslam
=========

A ROS package that implements a multi-robot SLAM system.
For full info see original repo [cg_mrslam](https://github.com/mtlazaro/cg_mrslam)

#### What's new

Added new mode ``real2``. Robots publish all messages to ``<namespace>/mrslam_msgs_rX`` (where ``X`` - number of recipient) when there are any subscribers. The others robots automatically subscribe to the corresponding topic.  

It allows you
- To run multiple robots on single host and simulate them in Gazebo
- To run robots on different hosts and communicate them through [multimaster_fkie](https://github.com/fkie/multimaster_fkie)

Changed console output to show you only helpful info through ``rosconsole``. To show debug messages set ``#define DEBUG = true`` in ``definitions.h``

Added ``g2o_merger`` to merge output graphs of robots into one. After it you can open and optimize this graph in ``g2o_viewer``

#### How to use

First robot:
```
rosrun cg_mrslam cg_mrslam -idRobot 0 -nRobots 2 -scanTopic scan -mapFrame robot0/map -odomFrame robot0/odom -baseFrame robot0/base_link -modality real2 -publishMap -publishGraph -o graph.g2o __ns:=robot0
```
Second robot:
```
rosrun cg_mrslam cg_mrslam -idRobot 1 -nRobots 2 -scanTopic scan -mapFrame robot1/map -odomFrame robot1/odom -baseFrame robot1/base_link -modality real2 -publishMap -publishGraph -o graph.g2o __ns:=robot1
```
g2o_merger:
```
g2o_merger -f1 graph1.g2o -f2 graph2.g2o -o merged.g2o
```
