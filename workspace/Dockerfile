# Preamble from extension [nvidia]
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.04 as glvnd

FROM osrf/ros:melodic-desktop-full
USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=glvnd /usr/share/glvnd/egl_vendor.d/10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && apt-get install -y alsa-base pulseaudio

ARG TIAGO_USER
SHELL ["/bin/bash", "-c"]
RUN echo "${TIAGO_USER} ALL=(ALL) NOPASSWD:ALL" | EDITOR="tee -a" visudo

RUN apt-get update
RUN apt-get install wget vim openssh-server -y

RUN sed -i "s/Port 22/Port 2222/" /etc/ssh/sshd_config
COPY entrypoint.sh /entrypoint.sh

COPY pythonr /usr/bin/pythonr
RUN chown root.root /usr/bin/pythonr
RUN chmod 777 /usr/bin/pythonr

RUN mkdir -p /opt/tiago_public_ws
WORKDIR /opt/tiago_public_ws
RUN wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/kinetic-devel/tiago_public-melodic.rosinstall
RUN rosinstall src /opt/ros/melodic tiago_public-melodic.rosinstall

RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init
RUN rosdep update
RUN rosdep install  --from-paths src --ignore-src -y --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3 joint_impedance_trajectory_controller cartesian_impedance_controller omni_base_description omni_drive_controller" -y

RUN apt-get update
RUN apt-get install ros-melodic-catkin python-catkin-tools python-pip -y

RUN source /opt/ros/melodic/setup.bash; catkin build -DCATKIN_ENABLE_TESTING=0
