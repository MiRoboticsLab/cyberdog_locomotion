CYBERDOG LOCOMOTION
---

小米四足机器人运动控制代码库，基于[MIT Cheetah Software](https://github.com/mit-biomimetics/Cheetah-Software)开源项目构建，主要包括以下几个部分：
- common: 子模块共享代码
- control: 控制器代码
- hardware: 硬件相关代码
- simbridge: 外部仿真器接口代码
- scripts: 脚本工具

**警告**：该运动控制代码只在铁蛋2代上进行了充分测试。对于铁蛋1代，仅支持1代APP中除作揖和打滚之外的功能，且未经充分测试，请谨慎使用!

---
## 安装部署
安装部署分两种使用场景，一是仿真使用，二是实机部署。由于机器人机载算力有限，两种方式都需要在外部PC上编译，首先需要安装依赖：

### 安装依赖
- 安装Eigen
```
$ git clone https://gitlab.com/libeigen/eigen.git
$ cd eigen
$ git checkout 3.3.7
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
```
- 安装yaml-cpp
```
$ git clone https://github.com/jbeder/yaml-cpp.git
$ cd yaml-cpp
$ git checkout yaml-cpp-0.6.3
$ mkdir build
$ cd build
$ cmake -DYAML_BUILD_SHARED_LIBS=ON ..
$ make -j4
$ sudo make install
```
- 安装JAVA 11 (仅用于记录和查看lcm数据)
```
$ sudo apt install openjdk-11-jdk
```
- 安装lcm
```
$ git clone https://github.com/lcm-proj/lcm.git
$ cd lcm
$ mkdir build
$ cd build
$ cmake -DLCM_ENABLE_JAVA=ON ..
$ make
$ sudo make install
```

- 安装docker (仅用于交叉编译)

按照下面的链接所附步骤进行安装：

https://www.cnblogs.com/wt7018/p/11880666.html

或

https://docs.docker.com/engine/install/ubuntu/

然后给docker设置root权限：
```
$ sudo groupadd docker
$ sudo usermod -aG docker $USER
```

- 下载所需docker镜像
```
$ wget https://cdn.cnbj2m.fds.api.mi-img.com/os-temp/loco/loco_arm64_20220118.tar
$ docker load --input loco_arm64_20220118.tar
$ docker images
```

### 仿真部署
在外部PC本地编译，仅限用于本地调试开发，其生成的可执行文件一般**不能**直接发送到机器人上运行。首先下载本代码仓：
```
$ git clone https://github.com/MiRoboticsLab/cyberdog_locomotion.git
```
然后进行编译：
```
$ cd cyberdog_locomotion
$ mkdir build
$ cd build
$ cmake ..
$ make -j4
```
注：编译过程中如遇到**fatal error: GL/glut.h: No such file or directory**，可安装：
```
$ sudo apt-get install freeglut3-dev
```

以上流程可用于检测代码的完整性和可编译性。如需启用Gazebo仿真，可下载cyberdog_sim代码仓（包含此运控代码）并通过colcon进行ROS2环境下的编译，详见cyberdog_sim仓的ReadMe：
```
$ git clone https://github.com/MiRoboticsLab/cyberdog_sim.git
$ cd cyberdog_sim
$ vcs import < cyberdog_sim.repos
```

下载完代码后，将src/cyberdog_locomotion/CMakeLists.txt中的BUILD_ROS置为ON后进行编译：
```
$ source /opt/ros/galactic/setup.bash
$ colcon build --merge-install --symlink-install --packages-up-to cyberdog_locomotion cyberdog_ros2
```

编译完成后，在cyberdog_sim目录下通过python脚本即可运行仿真程序：
```
$ python3 src/cyberdog_ros2/cyberdog_gazebo/script/launchsim.py
```

### 实机部署
为了能使编译的文件可以直接在机器人上运行，需要在特定的docker镜像环境下进行交叉编译，具体步骤如下：
```
$ docker run -it --rm --name alan -v your_own_path/cyberdog_locomotion:/work/build_farm/workspace/cyberdog cr.d.xiaomi.net/athena/athena_cheetah_arm64:2.0 /bin/bash
$ cd /work/build_farm/workspace/cyberdog/
$ mkdir onboard-build
$ cd onboard-build
```
- 如要编译Cyberdog:
```
$ cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake -DONBOARD_BUILD=ON -DBUILD_FACTORY=ON -DBUILD_CYBERDOG2=OFF ..
$ make -j4
```
- 如要编译Cyberdog2:
```
$ cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake -DONBOARD_BUILD=ON -DBUILD_FACTORY=ON -DBUILD_CYBERDOG2=ON ..
$ make -j4
```
编译成功后, 将电脑连接至机器人的网口(Cyberdog2)或者Download USB口(Cyberdog)，然后可将生成的文件发送给机器人：
```
$ cd ~/cyberdog_locomotion/scripts
$ sudo ./scp_to_cyberdog.sh
```
文件发送成功后，运控程序会即刻生效运行。

---
## 数据分析
安装Python/Python3相关组件，下面以Python3为例：
```
$ sudo apt install python3
$ sudo apt install python3-pip
$ pip3 install numpy
$ pip3 install scipy
$ pip3 install lcm
```

在仿真/实际机器人运行时，可通过lcm工具进行数据分析：
```
$ cd ~/cyberdog_locomotion/scripts
$ ./make_types.sh # 生成数据类型，只需在首次或有新数据类型时执行
$ ./launch_lcm_spy.sh # 实时显示
$ ./launch_lcm_log.sh # 数据存储
$ ./launch_lcm_logplayer.sh #数据回放
```
---
## 开源要求
### 代码格式
代码格式由工程目录下.clang-format文件约定。使用方法如下：
```
$ sudo apt install clang-format # 安装clang-format
$ clang-format -i file_to_format.cpp # 更新指定文件格式
$ find . -regex '.*\.\(cpp\|hpp\|c\|h\)' -exec clang-format -style=file -i {} \; # 更新当前及其子目录下所有cpp, hpp, c, h文件格式。
```

### 代码风格
本代码遵循谷歌开源项目代码风格，具体参见[网址](https://zh-google-styleguide.readthedocs.io/en/latest/contents/)。

### 代码提交
为方便代码追溯，请遵循如下提交格式：
```
[Fix/New/Modify]: summary

JIRA-ID: N/A

1. commit message

Change-ID
Signed-off-by: xxx <xxx@company.com>
# Line 1：summary行，格式为：[提交类型]+[冒号]+[空格]+[简明该提交代码功能], 限制字符为80字符，不能换行
# Line 2：空行
# Line 3：JIRA-ID (没有就填N/A)
# Line 4：空行
# Line 5：进版原因(commit message)
# Line 6：空行
# Line 7: Change-Id行,git hooks会自动生成
# Line 8：Singed-off信息
```
需要注意的事项有：
- 用git commit -s指令来提交，以自动生成Signed-off信息;
- 提交类型只限Fix（Bug修复），New（新功能），Modify（改进代码）三种；
- commit message尽量写得详细，交代清楚问题，分析原因，避免和summary行雷同，支持中文。

为方便提交，可使用提交模板：
```
$ git config --global commit.template 当前文件夹地址/.commit-template
```
