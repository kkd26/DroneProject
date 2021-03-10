FROM mbyzhang/olympe-ros

RUN echo "yaml file:///etc/ros/rosdep/rules/alpha.yaml" > /etc/ros/rosdep/sources.list.d/50-alpha.list && \
    mkdir -p /etc/ros/rosdep/rules && \
    printf "python3-janus:\n    ubuntu: [python3-janus]\n" > /etc/ros/rosdep/rules/alpha.yaml && \
    rosdep update

ADD ros /opt/alpha_ws/src

RUN cd /opt/alpha_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    apt clean

RUN cd /opt/alpha_ws && \
    /ros_entrypoint.sh catkin_make

ADD docker/alpha_launch.sh /
CMD [ "/olympe-ros-entrypoint.sh", "/alpha_launch.sh"]
