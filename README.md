# Grasp System
## Introduction
Grasp system is implemented through RGB-D data captured by [Intel Realsense F200](https://software.intel.com/en-us/intel-realsense-sdk/download) on the [Dobot Magician](http://www.dobot.cc/) platform. This system adopt depth segmentation, color classification and points cloud registration framework to predict grasping region on common desktop objects, such as cup, can, teapot. 

## How to Control

### Preparation
Install the driver of [Dobot Magician](http://www.dobot.cc/) and run the [Dobot server](http://github.com/Ext4FAT/MechanicalArm)

### Operation
There are some hotkeys help us control[Dobot](http://github.com/Ext4FAT/MechanicalArm) before do automatic grasping work.
|Hotkey|Operation|
|:---:|:---:|
|Space|Find ChessBoard|
|ESC|Quit Client|
|g|Grasp|
|h|Home|
|q|Quit Server|
|k|Up|
|j|Down|
