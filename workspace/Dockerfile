# Preamble from extension [nvidia]
# Ubuntu 16.04 with nvidia-docker2 beta opengl support
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu16.04 as glvnd

# Preamble from extension [x11]


FROM osrf/ros:kinetic-desktop-full
USER root
# Snippet from extension [nvidia]
# Open nvidia-docker2 GL support
COPY --from=glvnd /usr/local/lib/x86_64-linux-gnu /usr/local/lib/x86_64-linux-gnu
COPY --from=glvnd /usr/local/lib/i386-linux-gnu /usr/local/lib/i386-linux-gnu
COPY --from=glvnd /usr/lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu
COPY --from=glvnd /usr/lib/i386-linux-gnu /usr/lib/i386-linux-gnu
# if the path is alreaady present don't fail because of being unable to append
RUN ( echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && ldconfig || grep -q /usr/local/lib/x86_64-linux-gnu /etc/ld.so.conf.d/glvnd.conf ) && \
    ( echo '/usr/local/lib/i386-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf &&  ldconfig || grep -q /usr/local/lib/i386-linux-gnu /etc/ld.so.conf.d/glvnd.conf )
ENV LD_LIBRARY_PATH /usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

COPY --from=glvnd /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json




ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Snippet from extension [x11]

ARG TIAGO_USER
SHELL ["/bin/bash", "-c"]
RUN echo "${TIAGO_USER} ALL=(ALL) NOPASSWD:ALL" | EDITOR="tee -a" visudo

RUN apt-get update
RUN apt-get install wget vim openssh-server -y

RUN sed -i "s/Port 22/Port 2222/" /etc/ssh/sshd_config
COPY entrypoint.sh /entrypoint.sh
RUN chmod 777 /entrypoint.sh


COPY pythonr /usr/bin/pythonr
RUN chown root.root /usr/bin/pythonr
RUN chmod 777 /usr/bin/pythonr

RUN mkdir -p /opt/tiago_public_ws
WORKDIR /opt/tiago_public_ws
COPY tiago_public_ws /opt/tiago_public_ws

RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init
RUN rosdep update --include-eol-distros
RUN rosdep install --from-paths src --ignore-src --rosdistro kinetic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3 joint_impedance_trajectory_controller urdf_test uvc_camera range_sensor_layer teb_local_planner pointcloud_to_laserscan octomap_server spatio_temporal_voxel_layer openni2_launch moveit_ros_visualization slam_toolbox sick_tim libuvc_camera mynteye_description stockbot_description" -y

RUN apt-get install ros-kinetic-teb-local-planner ros-kinetic-range-sensor-layer ros-kinetic-pointcloud-to-laserscan ros-kinetic-catkin python-pip python-catkin-tools -y

RUN pip install pip==20.3.4

RUN source /opt/ros/kinetic/setup.bash; catkin build -DCATKIN_ENABLE_TESTING=0
