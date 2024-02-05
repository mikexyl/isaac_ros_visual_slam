FROM isaac-vslam

# remove system installed isaac-ros-visual-slam
RUN apt-get remove -y ros-${ROS_DISTRO}-isaac-ros-visual-slam
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-isaac-ros-nitros && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    llvm clang && \
    rm -rf /var/lib/apt/lists/*

# Upgrade to get latest realsense package that includes d455 etc.
RUN apt update && apt upgrade -y && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
