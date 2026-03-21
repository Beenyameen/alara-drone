FROM introlab3it/rtabmap:20.04

USER root
RUN apt-get update && apt-get install -y libzmq3-dev && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . /app

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cmake . && make"

CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && ./reconstruction"]