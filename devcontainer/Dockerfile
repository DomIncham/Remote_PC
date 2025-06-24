FROM osrf/ros:humble-desktop-full

SHELL [ "/bin/bash" , "-c" ]



# Upgrade all packages
RUN sudo apt update && sudo apt upgrade -y


# Install essential packages
RUN sudo apt install -y wget

# Install dependant ROS2 Package
RUN sudo apt install -y ros-humble-gazebo-*
RUN sudo apt install -y ros-humble-cartographer
RUN sudo apt install -y ros-humble-cartographer-ros
RUN sudo apt install -y ros-humble-navigation2
RUN sudo apt install -y ros-humble-nav2-bringup
RUN sudo apt install ros-humble-gazebo-ros-pkgs    
RUN sudo apt install -y python3 python3-pip



# Create overlay workspace
WORKDIR /root/ros_ws/src
COPY packages.repos .

RUN vcs import < packages.repos; \
    cd ..; \
    rosdep install --from-paths src --ignore-src -r -y; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    colcon build;

# Add sourcing ROS setup.bash to .bashrc
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc
RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
RUN echo 'source /workspaces/ros2/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
RUN echo 'source /workspaces/ros2/assembed/install/setup.bash' >> ~/.bashrc
RUN echo 'source /workspaces/ros2/ros2_ws/install//setup.bash' >> ~/.bashrc
RUN echo 'source /workspaces/ros2/swerve_robot/install/setup.bash' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=/workspaces/ros2/gazebo_models' >> ~/.bashrc
RUN echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share' >> ~/.bashrc

#Add X11 Server
RUN apt-get install -y x11-apps
RUN apt-get install -y x11-utils

# Upgrade all packages
RUN sudo apt update && sudo apt upgrade -y

 
RUN sudo apt-get install -y ros-humble-urdf-tutorial
RUN sudo apt install -y ros-humble-rqt-robot-steering
RUN sudo apt install -y ros-humble-teleop-twist-keyboard
RUN sudo apt install -y ros-humble-xacro
RUN sudo apt install -y ros-humble-joint-state-publisher
RUN sudo apt-get install -y ros-humble-tf-transformations
RUN sudo apt install -y ros-humble-robot-localization
RUN sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
RUN pip3 install pyserial
RUN sudo apt install python3-pip -y
RUN pip3 install opencv-python
RUN sudo apt install qtwayland5 -y
RUN pip install opencv-contrib-python
RUN sudo apt install nano