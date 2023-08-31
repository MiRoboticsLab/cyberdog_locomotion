# **Dockerfile使用说明**

### **一、Dockerfile内容**

##### 1.选择ubuntu:16.04为基础镜像

```Bash
#指定基础镜像
FROM ubuntu:16.04
#指定作者
MAINTAINER dongwuming "dongwuming@xiaomi.com"
#创建工作目录和交叉编译目录
RUN mkdir -p  /home/builder /usr/xcc
#指定工作路径
WORKDIR /home/builder
```

**2.APT安装下载工具**

```Bash
RUN apt-get update \
&& apt-get install -y \
   mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev git vim wget libjchart2d-java doxygen liblua5.3-dev lua5.3 python3-dev python3-pip \
&& apt-get clean --yes
```

**3.安装cmake 3.13.2版本**

```Bash
RUN wget http://www.cmake.org/files/v3.13/cmake-3.13.2.tar.gz \
&& tar -zxf cmake-3.13.2.tar.gz \
&& cd cmake-3.13.2 \
&& ./configure \
&& make -j8 \
&& make install
```

**4.安装Eigen**

```Bash
RUN git clone https://gitlab.com/libeigen/eigen.git \
&& cd eigen \
&& git checkout 3.3.7 \
&& mkdir build \
&& cd build \
&& cmake .. \
&& make 安装Eigen
```

**5.安装yaml-cpp**

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

**6.安装lcm**

```Bash
RUN git clone https://github.com/lcm-proj/lcm.git \
&& cd lcm \
&& mkdir build \
&& cd build \
&& cmake .. \
&& make -j8 \
&& make install
```

**7.安装交叉编译工具**

```Bash
RUN wget https://os-temp.cnbj2m.mi-fds.com/os-temp/devops/aarch64-linux-gnu.tgz \
&& tar -zxf aarch64-linux-gnu.tgz -C /usr/xcc
```

**8.设置环境变量**

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

### **二、镜像制作流程**

**1.本机安装好docker环境**


**2.新建一个空文件夹，将Dockerfile文件复制到这个空目录中**

```Bash
mkdir loco_ubuntu
cp Dockerfile loco_ubuntu/
```

**3.在 Dockerfile 文件所在目录执行：**

```Bash
docker build -t 镜像名:tag .
例如：docker build -t loco_ubuntu:1.0.0 .
```

### **三、 docker 编译运控代码**

**1.创建容器并建立本地目录到镜像目录的映射关系**

```Bash
docker run -it -v /本地目录:/镜像中的目录 镜像名:tag bash
#这里xxx代表你的用户名
例如：docker run -it -v /home/xxx/cyberdog_locomotion:/home/builder/cyberdog_locomotion loco_ubuntu:1.0.0 bash
```

**2.在docker终端进入到/home/builder/cyberdog_locomotion目录下执行：**

```Bash
cd /home/builder/cyberdog_locomotion
mkdir onboard-build
cd onboard-build
cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake -DONBOARD_BUILD=ON -DBUILD_FACTORY=ON -DBUILD_CYBERDOG2=ON ..
make -j8
```
