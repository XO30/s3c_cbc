FROM ros:iron

RUN apt-get update && apt-get install -y \
    python3-pip \
    ffmpeg \
    libsm6 \
    libxext6 \
    curl \
    gnupg2 \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update

WORKDIR /root
COPY ./src/part_detection_pkg/requirements.txt .
RUN pip3 install -r requirements.txt

RUN apt-get update --fix-missing && apt install -y ros-$ROS_DISTRO-librealsense2* ros-$ROS_DISTRO-realsense2-* ros-$ROS_DISTRO-rviz2

RUN pip3 install scikit-image

WORKDIR /root
COPY ./run.sh /root/run.sh
RUN chmod +x /root/run.sh