FROM tyoung96/multi-turtlebot-gazebo

SHELL ["/bin/bash", "-c"]

# Install dependencies

ENV RCUTILS_COLORIZED_OUTPUT=1

RUN apt-get update && apt-get install -y \ 
    git python3-pip

RUN echo "source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash" >> ~/.bashrc


ENV TURTLEBOT3_MODEL=burger

RUN apt install psmisc

RUN history -s ros2 launch turtlebot3_gazebo multi_turtlebot3_world.launch.py

RUN apt-get update && apt-get install -y \
    ros-humble-joint-state-publisher-gui \
    ros-humble-image-transport-plugins \
    ros-humble-rqt-image-view \
    ros-humble-rqt-tf-tree

RUN pip3 install opencv-python opencv-contrib-python transforms3d
RUN apt-get update && apt-get install -y \
    ros-humble-tf-transformations
 
RUN apt-get update && apt-get install -y \ 
    ros-humble-navigation2 \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-slam-toolbox

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y \
    ros-humble-robot-localization

RUN pip3 install py-trees
RUN apt-get update && apt-get install -y \
    ros-humble-py-trees-ros \
    libqt5webengine5 libqt5webenginewidgets5 python3-pyqt5.qtwebengine
RUN pip install pyqtwebengine
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs
ENV ROS_DOMAIN_ID=13

ENV GAZEBO_MODEL_PATH=/workspace/src/turtlebot3_simulations/turtlebot3_gazebo/models/
# Set the working directory
WORKDIR /workspace
