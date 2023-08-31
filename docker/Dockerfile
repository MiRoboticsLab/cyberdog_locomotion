FROM ubuntu:16.04
#Specified Author
MAINTAINER dongwuming "dongwuming@xiaomi.com"
#Create work path and aarch64-linux-gnu path
RUN mkdir -p  /home/builder /usr/xcc
#Specified work path
WORKDIR /home/builder

#Apt install build tool
RUN apt-get update \
&& apt-get install -y \
   mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev git vim wget libjchart2d-java doxygen liblua5.3-dev lua5.3 python3-dev python3-pip \
&& apt-get clean --yes

#Install cmake 3.13.2 version
RUN wget http://www.cmake.org/files/v3.13/cmake-3.13.2.tar.gz && tar -zxf cmake-3.13.2.tar.gz && cd cmake-3.13.2 && ./configure && make -j8 && make install

#Install Eigen
RUN git clone https://gitlab.com/libeigen/eigen.git && cd eigen && git checkout 3.3.7 && mkdir build && cd build && cmake .. && make install

#Install yaml-cpp
RUN git clone https://github.com/jbeder/yaml-cpp.git && cd yaml-cpp && git checkout yaml-cpp-0.6.3 && mkdir build && cd build && cmake -DYAML_BUILD_SHARED_LIBS=ON .. && make -j8 && make install

#Install lcm
RUN git clone https://github.com/lcm-proj/lcm.git && cd lcm && mkdir build && cd build && cmake .. && make -j8 && make install

#Install aarch64-linux-gnu
RUN wget https://os-temp.cnbj2m.mi-fds.com/os-temp/devops/aarch64-linux-gnu.tgz && tar -zxf aarch64-linux-gnu.tgz -C /usr/xcc

#Setting environment variables:
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV AR=/usr/xcc/aarch64-openwrt-linux-gnu/bin/aarch64-openwrt-linux-gnu-ar
ENV AS=/usr/xcc/aarch64-openwrt-linux-gnu/bin/aarch64-openwrt-linux-gnu-as
ENV CC=/usr/xcc/aarch64-openwrt-linux-gnu/bin/aarch64-openwrt-linux-gnu-gcc
ENV CMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake
ENV CPP=/usr/xcc/aarch64-openwrt-linux-gnu/bin/aarch64-openwrt-linux-gnu-cpp
ENV CROSS_ROOT=/usr/xcc/aarch64-openwrt-linux-gnu
ENV CXX=/usr/xcc/aarch64-openwrt-linux-gnu/bin/aarch64-openwrt-linux-gnu-g++
ENV FC=/usr/xcc/aarch64-openwrt-linux-gnu/bin/aarch64-openwrt-linux-gnu-gfortran
ENV LD=/usr/xcc/aarch64-openwrt-linux-gnu/bin/aarch64-openwrt-linux-gnu-ld
ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/xcc/aarch64-unknown-linux-gnueabi/bin:/usr/xcc/aarch64-openwrt-linux-gnu/bin
ENV PKG_CONFIG_PATH=/usr/xcc/aarch64-openwrt-linux-gnu/aarch64-openwrt-linux-gnu/usr/lib/pkgconfig:/usr/xcc/aarch64-openwrt-linux-gnu/aarch64-openwrt-linux-gnu/usr/share/pkgconfig
ENV QEMU_LD_PREFIX=/usr/xcc/aarch64-unknown-linux-gnueabi/aarch64-unknown-linux-gnueabi/sysroot
ENV QEMU_SET_ENV=LD_LIBRARY_PATH=/usr/xcc/aarch64-unknown-linux-gnueabi/lib:/usr/xcc/aarch64-unknown-linux-gnueabi/aarch64-unknown-linux-gnueabi/sysroot
ENV STAGING_DIR=/usr/xcc/aarch64-openwrt-linux-gnu/aarch64-openwrt-linux-gnu

CMD ["/bin/bash"]
