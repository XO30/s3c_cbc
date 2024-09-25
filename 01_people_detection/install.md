# Installation
This document shows how People Detection can be set up.

## Prerequisites
All the software that is needed is already in the Docker image. The image must therefore be built initially. This could take a while, as OpenCV has to be built from source.

After the successful build, the container can be started via the Compose file or alternatively via a script.

## Start up
The container can be started manually, via a script or automatically by the compose file when the host is started. In any case, there is a critical component. It must be ensured that the Ricoh Theta Z1 is running and in live mode before the Sontainer is started. If this is not the case, the People Detection node cannot start correctly.

# LIDAR

1. Destination IP Address on 192.168.1.200 web interface
2. firewall ports 6699 7788
3. build and run

# General information on the components

## Intel Realsense
To get al live view of the camera stream:

https://github.com/IntelRealSense/realsense-ros#installation

1. Install and updates with realsense viewer
2. ev "sudo udevadm control --reload-rules && sudo udevadm trigger"
3. roslaunch realsense_camera rs_camera.launch

## Richoh Theta Z1
Each time the camera is started, it must be ensured that it is in live mode

# Theta Z1 Remote Control
Download and unzip https://sourceforge.net/projects/libptp/
go to containing folder
./configure
make
sudo make install
sudo ldconfig -v
ptpcam --show-property=0x5013 --val=0x8005

Video tutorial: https://www.youtube.com/watch?v=HfwSFvp5jXY