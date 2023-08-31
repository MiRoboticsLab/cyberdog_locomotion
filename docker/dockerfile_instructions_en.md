# **Dockerfile Instructions**

### **1.Dockerfile content**

##### 1.Select ubuntu:16.04 as the base image

```Bash
#Specified base image
FROM ubuntu:16.04
#Specified Author
MAINTAINER dongwuming "dongwuming@xiaomi.com"
#Create work path and aarch64-linux-gnu path
RUN mkdir -p  /home/builder /usr/xcc
#Specified work path
WORKDIR /home/builder
```

**2.Install and download tools**

```Bash
RUN apt-get update \
&& apt-get install -y \
   mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev git vim wget libjchart2d-java doxygen liblua5.3-dev lua5.3 python3-dev python3-pip \
&& apt-get clean --yes
```

**3.Install cmake 3.13.2 version**

```Bash
RUN wget http://www.cmake.org/files/v3.13/cmake-3.13.2.tar.gz \
&& tar -zxf cmake-3.13.2.tar.gz \
&& cd cmake-3.13.2 \
&& ./configure \
&& make -j8 \
&& make install
```

**4.Install Eigen**

```Bash
RUN git clone https://gitlab.com/libeigen/eigen.git \
&& cd eigen \
&& git checkout 3.3.7 \
&& mkdir build \
&& cd build \
&& cmake .. \
&& make install
```

**5.Install yaml-cpp**

```Bash
RUN git clone https://github.com/jbeder/yaml-cpp.git \
&& cd yaml-cpp \
&& git checkout yaml-cpp-0.6.3 \
&& mkdir build \
&& cd build \
&& cmake -DYAML_BUILD_SHARED_LIBS=ON .. \
&& make -j13 \
&& make install
```

**6.Install lcm**

```Bash
RUN git clone https://github.com/lcm-proj/lcm.git \
&& cd lcm \
&& mkdir build \
&& cd build \
&& cmake .. \
&& make -j8 \
&& make install
```

**7.Install aarch64-linux-gnu**

```Bash
RUN wget https://os-temp.cnbj2m.mi-fds.com/os-temp/devops/aarch64-linux-gnu.tgz \
&& tar -zxf aarch64-linux-gnu.tgz -C /usr/xcc
```

**8.Setting environment variables**

```Bash
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
```

### **2.Image making process**

**1.Install Docker environment on your machine**


**2.Create an empty folder and copy the Dockerfile file into it**

```Bash
mkdir loco_ubuntu
cp Dockerfile loco_ubuntu/
```

**3.Execute in the directory where the Dockerfile file resides:**

```Bash
docker build -t ImageName:tag .
For example：docker build -t loco_ubuntu:1.0.0 .
```

### **docker compiles cyberdog_locomotion**

**1.Create a container and create a mapping between a local directory and an image directory**

```Bash
docker run -it -v /LocalDirectory:/DirectoryImageName ImageName:tag bash
For example：docker run -it -v /home/xxx/cyberdog_locomotion:/home/builder/cyberdog_locomotion loco_ubuntu:1.0.0 bash
```

**2.Enter the /home/builder/cyberdog_locomotion on docker terminal and execute:**

```Bash
cd /home/builder/cyberdog_locomotion
mkdir onboard-build
cd onboard-build
cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake -DONBOARD_BUILD=ON -DBUILD_FACTORY=ON -DBUILD_CYBERDOG2=ON ..
make -j8
```
