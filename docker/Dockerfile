FROM tyoung96/multi-turtlebot-gazebo

SHELL ["/bin/bash", "-c"]

# Install dependencies

ENV RCUTILS_COLORIZED_OUTPUT=1

RUN apt-get update && apt-get install -y \ 
    git python3-pip

RUN echo "source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash" >> ~/.bashrc

ENV TURTLEBOT3_MODEL=burger

RUN apt install psmisc

ENV GAZEBO_MODEL_PATH=/workspace/src/turtlebot3_simulations/turtlebot3_gazebo/models/
# Set the working directory
WORKDIR /workspace
