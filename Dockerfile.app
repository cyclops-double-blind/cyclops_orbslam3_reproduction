FROM ros:melodic-ros-core-bionic

RUN apt-get update -y \
    && apt-get install -y --no-install-recommends gosu \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

COPY --from=orbslam3-docker-build \
    /opt/ros/melodic/lib/libcv_bridge.so /opt/ros/melodic/lib/libcv_bridge.so
COPY --from=orbslam3-docker-build \
    /opt/ros/melodic/share/cv_bridge/package.xml \
    /opt/ros/melodic/share/cv_bridge/package.xml

RUN apt-get update -y \
    && apt-get install -y --no-install-recommends \
      libopencv-core3.2 libopencv-imgproc3.2 libopencv-imgcodecs3.2 \
      libopencv-calib3d3.2 libopencv-features2d3.2 libopencv-highgui3.2 \
      libopengl0 libgl1 libglx0 libglew2.0 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -y \
    && apt-get install -y --no-install-recommends \
        avahi-daemon avahi-discover libnss-mdns \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws
COPY install/lib /usr/lib
COPY src/catkin/install/* /catkin_ws/

COPY entry/init.sh /entry/init.sh
COPY entry/entry.sh /entry/entry.sh
ENTRYPOINT ["/entry/entry.sh"]
