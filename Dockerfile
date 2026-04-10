FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-mavros-msgs \
    ros-jazzy-usb-cam \
    ros-jazzy-image-transport \
    ros-jazzy-compressed-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-std-srvs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets for MAVROS
RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh || true

# Install Python deps
RUN pip3 install --break-system-packages pymavlink

# Set up workspace
WORKDIR /ros_ws
COPY src/ src/

# Build all onboard packages (rov_flight, rov_photogrammetry)
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# DDS config
COPY cyclonedds_onboard.xml /ros_ws/cyclonedds.xml

# Environment
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///ros_ws/cyclonedds.xml

# Entrypoint
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
