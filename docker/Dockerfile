# rich base image
FROM osrf/ros:humble-desktop

# install development tools
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    bash-completion \
    htop

# copy project to container temporarily for installing dependencies
COPY src /home/src
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep update --rosdistro=humble && \
    rosdep install -y --from-paths /home/src --ignore-src --rosdistro humble"

# install dependencies
COPY Makefile /home/Makefile
RUN cd /home && make install_deps

# remove /home/src and /home/Makefile after installing dependencies
RUN rm -rf /home/src
RUN rm /home/Makefile

# set entrypoint including process of sourcing ros environment
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# set default command
CMD ["/bin/bash"]
