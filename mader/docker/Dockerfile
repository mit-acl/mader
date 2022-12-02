FROM ros:noetic

LABEL Maintainer="Kota Kondo <kkondo@mit.edu>" \
      Description="Ubuntu 20.04, ROS Noetic for MADER"

RUN apt-get update && \
      apt-get upgrade -y && \
      apt-get install -q -y --no-install-recommends git tmux vim apt-utils && \
      apt-get install -q -y --no-install-recommends wget && \
      apt-get autoremove -y && \
      apt-get clean && \
      rm -rf /var/lib/apt/lists/*

ENV ROS_DIR=/opt/ros/${ROS_DISTRO}

## gurobi
RUN wget https://packages.gurobi.com/9.5/gurobi9.5.2_linux64.tar.gz -P . && \
      tar -xzf gurobi9.5.2_linux64.tar.gz && \
      rm gurobi9.5.2_linux64.tar.gz && \
      mv gurobi952/ /opt && \
# RUN echo >> ~/.bashrc
# RUN echo export GUROBI_HOME="/opt/gurobi952/linux64"  >> ~/.bashrc
# RUN echo export PATH="${PATH}:${GUROBI_HOME}/bin"  >> ~/.bashrc
# RUN echo export LD_LIBRARY_PATH="${GUROBI_HOME}/lib"  >> ~/.bashrc
      . /opt/ros/noetic/setup.sh 
WORKDIR /opt/gurobi952/linux64/src/build
RUN apt-get update && apt-get install make && \
      apt install build-essential -y --no-install-recommends && \
      make && \
      cp libgurobi_c++.a ../../lib/ && \
      . /opt/ros/noetic/setup.sh

# # install MADER
RUN mkdir -p /home/mader_ws/src
WORKDIR /home/mader_ws/src
RUN git clone https://github.com/mit-acl/mader.git && \
      apt-get install python-is-python3 -y
WORKDIR /home/mader_ws/
RUN echo source /opt/ros/noetic/setup.bash >> ~/.bashrc
RUN apt-get install python-is-python3 -y
WORKDIR /home/mader_ws/

# the code below is the replacement of "RUN bash src/mader/install_and_compile.sh"
#INSTALL CGAL v4.14.2
##########################################
RUN apt-get install libgmp3-dev libmpfr-dev -y
RUN mkdir -p /home/installations/cgal
WORKDIR /home/installations/cgal
RUN wget https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-4.14.2/CGAL-4.14.2.tar.xz
RUN tar -xf CGAL-4.14.2.tar.xz
WORKDIR CGAL-4.14.2/
RUN cmake . -DCMAKE_BUILD_TYPE=Release
RUN make install

#INSTALL python-catkin-tools (to be able to use catkin build)
##########################################
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - |  apt-key add -
RUN apt-get install python3-catkin-tools -y

#CLONE SUBMODULES, INSTALL DEPENDENCIES AND COMPILE
##########################################
WORKDIR /home/mader_ws/src/mader
RUN git submodule init && git submodule update
WORKDIR /home/mader_ws
RUN DEBIAN_FRONTEND=noninteractive rosdep install --from-paths /home/mader_ws/src --ignore-src -r -y
RUN catkin config -DCMAKE_BUILD_TYPE=Release
RUN export GUROBI_HOME="/opt/gurobi952/linux64" && \
    export PATH="${PATH}:${GUROBI_HOME}/bin" && \
    export LD_LIBRARY_PATH="${GUROBI_HOME}/lib" && \
    . /opt/ros/noetic/setup.sh && catkin build
RUN echo source /home/mader_ws/devel/setup.bash >> ~/.bashrc