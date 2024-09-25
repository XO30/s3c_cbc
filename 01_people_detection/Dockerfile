# Use ROS Noetic base image
FROM ros:noetic

# Update and upgrade packages
RUN apt-get update && apt-get upgrade -y

# Install GStreamer and other dependencies for the RICOH THETA Z1
RUN apt-get install -y \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-doc \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev \
    libjpeg-dev \
    libusb-1.0 


# Install general stuff
RUN apt-get install -y \
    git \
    usbutils \
    python3-pip \
    nano

# Install dependencies for the LiDAR
RUN apt-get install -y \
    libpcap-dev \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-cv-bridge

# Create a workspace for the RICOH THETA Z1
WORKDIR /theta_z1

# Clone and build libuvc-theta from GitHub
RUN git clone https://github.com/ricohapi/libuvc-theta.git && \
    cd libuvc-theta && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    make install

# Clone and build gstthetauvc from GitHub
RUN git clone https://github.com/nickel110/gstthetauvc.git && \
    cd gstthetauvc/thetauvc && \
    make && \
    cp gstthetauvc.so /usr/lib/x86_64-linux-gnu/gstreamer-1.0/

# Update library paths and GStreamer plugin paths
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/libuvc.conf && ldconfig
ENV GST_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gstreamer-1.0

WORKDIR /cv

RUN pip uninstall -y opencv-python opencv-python-headless
RUN git clone https://github.com/opencv/opencv-python.git 
WORKDIR /cv/opencv-python
ENV ENABLE_CONTRIB=0
ENV ENABLE_HEADLESS=1
ENV CMAKE_ARGS="-DWITH_GSTREAMER=ON"
RUN python3 -m pip wheel . --verbose
RUN python3 -m pip install opencv_python*.whl

RUN apt-get update && apt-get install ros-noetic-rviz libyaml-cpp-dev -y

# Cleanup
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Install aica-api
RUN pip install aica-api==v2.0.0

# Switch back to the root directory
WORKDIR /root

# Install dependencies for the people detection
COPY people_detection/people_detection/yolov7/requirements.txt requirements.txt
RUN pip install -r requirements.txt

# Create a ROS workspace
WORKDIR /root/catkin_ws/src

# Copy the people detection source code
COPY people_detection ./

# Switch back to the catkin workspace
WORKDIR /root/catkin_ws

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"

WORKDIR /root/
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
