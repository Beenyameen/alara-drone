FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git cmake build-essential \
    libglew-dev libeigen3-dev \
    libpython2.7-dev pkg-config wget \
    libssl-dev libcanberra-gtk-module libcanberra-gtk3-module \
    x11-apps \
    libboost-serialization-dev libboost-system-dev \
    libzmq3-dev liblz4-dev \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://raw.githubusercontent.com/zeromq/cppzmq/v4.3.0/zmq.hpp -O /usr/local/include/zmq.hpp

WORKDIR /opt
RUN wget https://github.com/opencv/opencv/archive/4.4.0.tar.gz && \
    tar -xzf 4.4.0.tar.gz && \
    mkdir -p opencv-4.4.0/build && cd opencv-4.4.0/build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /opt/4.4.0.tar.gz /opt/opencv-4.4.0

WORKDIR /opt
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git checkout v0.6 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

WORKDIR /opt
RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
WORKDIR /opt/ORB_SLAM3
RUN chmod +x build.sh && ./build.sh

WORKDIR /app
COPY . /app
RUN cmake . && make -j$(nproc)

CMD ["./trajectory"]