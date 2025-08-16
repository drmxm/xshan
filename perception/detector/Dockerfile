# Jetson JP 6.x / L4T r36.x
FROM nvcr.io/nvidia/l4t-base:r36.2.0

SHELL ["/bin/bash","-lc"]
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    TZ=Etc/UTC

# ROS 2 Humble + cv_bridge + vision_msgs + OpenCV (+ cascade data)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg2 lsb-release dirmngr locales tzdata python3-pip \
    && echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && locale-gen \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
         > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      ros-humble-vision-msgs \
      ros-humble-cv-bridge \
      ros-humble-image-transport \
      python3-opencv \
      opencv-data \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Auto-source ROS for interactive shells
RUN printf 'source /opt/ros/${ROS_DISTRO}/setup.bash\n' > /etc/profile.d/ros2.sh

WORKDIR /work
COPY start.sh /start.sh
COPY detector_node.py /detector_node.py
RUN chmod +x /start.sh

ENTRYPOINT ["/start.sh"]
