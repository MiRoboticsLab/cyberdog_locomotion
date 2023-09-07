CYBERDOG LOCOMOTION
---
Locomotion control algorithm for quadrupedal robots of Xiaomi Robotics Lab, based on the MIT open-source project [Cheetah Software](https://github.com/mit-biomimetics/Cheetah-Software), mainly includes 5 modules as below:
- common: shared code for all other modules
- control: control algorithm
- hardware: hardware-related code
- simbridge: interface code for bridging to external simulators
- scripts: useful script tools

**Warning**： this control algorithm was only fully tested on Cyberdog2. Regarding Cyberdog1, not all functionalities are guaranteed, so please be cautious to use!

---
## Deploy
There are two scenarios to deploy the code, one is in simulation and the other on real robots. Considering the limited computing resources of robot embedded PC, the code is always preferred to be compiled on powerful external PC in both use cases. So, first, you need to install dependencies on the external PC:

### Install Dependencies
- Eigen
```
$ git clone https://gitlab.com/libeigen/eigen.git
$ cd eigen
$ git checkout 3.3.7
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
```
- yaml-cpp
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
- JAVA 11 (only for lcm data log and review)
```
$ sudo apt install openjdk-11-jdk
```
- lcm
```
$ git clone https://github.com/lcm-proj/lcm.git
$ cd lcm
$ mkdir build
$ cd build
$ cmake -DLCM_ENABLE_JAVA=ON ..
$ make
$ sudo make install
```

- docker (only for cross-compiling)

Follow the steps described in this link to set up docker:

https://docs.docker.com/engine/install/ubuntu/

and then give the root permission to docker:
```
$ sudo groupadd docker
$ sudo usermod -aG docker $USER
```

- download docker image
```
$ wget https://cdn.cnbj2m.fds.api.mi-img.com/os-temp/loco/loco_arm64_20220118.tar
$ docker load --input loco_arm64_20220118.tar
$ docker images
```
or you can make the docker image accordting to the attached [DockerFile](./docker/Dockerfile). For more details, please refer to the [instruction document](./docker/dockerfile_instructions_en.md).

### Simulation
Compile on the external PC, and the compiled product is only for simulation usage, **CAN NOT** be deployed on the real robot. First, download the code:
```
$ git clone https://github.com/MiRoboticsLab/cyberdog_locomotion.git
```
And then compile：
```
$ cd cyberdog_locomotion
$ mkdir build
$ cd build
$ cmake ..
$ make -j4
```
Note： if encounter **fatal error: GL/glut.h: No such file or directory**，try to install：
```
$ sudo apt-get install freeglut3-dev
```
Above is for checking if the code is integral and compilable. To run simulation in Gazebo, the whole [cyberdog_sim](https://github.com/MiRoboticsLab/cyberdog_sim) repository should be downloaded which already includes this code inside and compiled in ROS2 environment. Please refer to the ReadMe of the [cyberdog_sim](https://github.com/MiRoboticsLab/cyberdog_sim) project for more details:
```
$ git clone https://github.com/MiRoboticsLab/cyberdog_sim.git
$ cd cyberdog_sim
$ vcs import < cyberdog_sim.repos
```

Then, the BUILD_ROS should be set to ON in src/cyberdog_locomotion/CMakeLists.txt, and all the project can be compiled in the directory 'cyberdog_sim':
```
$ source /opt/ros/galactic/setup.bash 
$ colcon build --merge-install --symlink-install --packages-up-to cyberdog_locomotion cyberdog_ros2
```

Run the simulation with a python script:
```
$ python3 src/cyberdog_ros2/cyberdog_gazebo/script/launchsim.py
```


### Real Robot
In order to allow compiled products to run directly on real robots, you need to do cross-compilation in a specific docker environment:
```
$ docker run -it --rm --name alan -v your_own_path/cyberdog_locomotion:/work/build_farm/workspace/cyberdog cr.d.xiaomi.net/athena/athena_cheetah_arm64:2.0 /bin/bash
$ cd /work/build_farm/workspace/cyberdog/
$ mkdir onboard-build
$ cd onboard-build
```
**Note**: your_own_path/cyberdog_locomotion needs to be an absolute path.
- For Cyberdog:
```
$ cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake -DONBOARD_BUILD=ON -DBUILD_FACTORY=ON -DBUILD_CYBERDOG2=OFF ..
$ make -j4
```
- For Cyberdog2:
```
$ cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake -DONBOARD_BUILD=ON -DBUILD_FACTORY=ON -DBUILD_CYBERDOG2=ON ..
$ make -j4
```
After successful compilation, connect the external PC to the Ethernet port of robot(Cyberdog2) or Download USB port(Cyberdog)，and then send the compiled product to robot：
```
$ cd ~/cyberdog_locomotion/scripts
$ sudo ./scp_to_cyberdog.sh
```
The control algorithm will automatically restart and run after the file transmission.

---
## Data Analysis
Install Python/Python3 related modules, take Python3 as example：
```
$ sudo apt install python3
$ sudo apt install python3-pip
$ pip3 install numpy
$ pip3 install scipy
$ pip3 install lcm
```

For both simulation and real robot, you can always use the lcm tools to perform data logging and analysis as mentioned below:
```
$ cd ~/cyberdog_locomotion/scripts
$ ./make_types.sh # generate data type descriptions，only needed for the first time or when new data types are added
$ ./launch_lcm_spy.sh # display real-time data
$ ./launch_lcm_log.sh # log data
$ ./launch_lcm_logplayer.sh # replay logged data
---
```
## Open-source Requirements
### Code Format
Coding format is defined by the file '.clang-format', how to use:
```
$ sudo apt install clang-format # install clang-format
$ clang-format -i file_to_format.cpp # specify which file to format
$ find . -regex '.*\.\(cpp\|hpp\|c\|h\)' -exec clang-format -style=file -i {} \; # format all the files ending with .cpp, .hpp, .c and .h under the current directory and its sub-directories
```

### Code Style
The repository follows Google Style. For more details, please refer to [website](https://zh-google-styleguide.readthedocs.io/en/latest/contents/)。

### Code Submit
For easier tracking of modification, please follow the submission format below:
```
[Fix/New/Modify]: summary

JIRA-ID: N/A

1. commit message

Change-ID
Signed-off-by: xxx <xxx@company.com>
```
To be noted:
- suggest using the command 'git commit -s' to submit so that signed-off information can be automatically generated;
- submit types can only be chosen from three: Fix(bug fixing), New(new feature)，Modify(code modify);
- please describe more details in the commit message and avoid repeating the summary line.

We also provided a commit template for your convenience：
```
$ git config --global commit.template ./.commit-template
```