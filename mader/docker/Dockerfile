from ros:noetic

LABEL Maintainer="Kota Kondo <kkondo@mit.edu>" \
      Description="Ubuntu 20.04, ROS Noetic, C++/Python3 for MADER"

RUN apt-get update -y && apt-get install -y \
    git \
    libeigen3-dev \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-pip 
    # python3-opencv \
    # ros-noetic-tf \
    # ros-noetic-cv-bridge

# be okay with python -> python3
RUN update-alternatives --install /usr/bin/python python $(which python3) 1

# install MADER
RUN git clone -b mindist https://github.com/mit-acl/clipper.git && \
    mkdir clipper/build && cd clipper/build && \
    cmake .. && make && \
    cd bindings/python && python3 -m pip install .

# # install python dependencies
# RUN python3 -m pip install \
#     opencv-python \
#     opencv-contrib-python \
#     open3d \
#     ipdb

# # setup frame-alignment code
# RUN mkdir -p catkin_ws/src && cd catkin_ws && \
#     catkin init && \
#     catkin config --extend /opt/ros/noetic
# COPY frame_alignment /catkin_ws/src/frame_alignment
# COPY frame_alignment_msgs /catkin_ws/src/frame_alignment_msgs

# # build frame-alignment code
# RUN cd catkin_ws && catkin build

# COPY frame_alignment/.docker/ros_entrypoint.sh /
# RUN chmod +x /ros_entrypoint.sh
# ENTRYPOINT ["/ros_entrypoint.sh"]
# WORKDIR /catkin_ws
