FROM ros:melodic-ros-core-bionic

RUN apt-get update -y \
    && apt-get install -y --no-install-recommends gosu \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

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

RUN apt-get update -y \
    && apt-get install -y --no-install-recommends \
        ros-melodic-cv-bridge ros-melodic-image-transport \
        ros-melodic-tf2-ros \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws
COPY install/lib /usr/lib
COPY install/share/ORBvoc.txt /ORBvoc.txt
COPY src/catkin/install/* /catkin_ws/

COPY etc/init.sh /etc/init.sh
COPY etc/entry.sh /etc/entry.sh
ENTRYPOINT ["/etc/entry.sh"]
