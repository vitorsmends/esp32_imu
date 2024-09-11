FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    bluez \
    bluez-tools \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pyserial

WORKDIR /ros2_ws

COPY . /ros2_ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y && \
    colcon build"

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    sudo rfcomm bind /dev/rfcomm0 A0:A3:B3:89:25:A6 && \
    ros2 launch esp32_imu imu.launch.py"]

