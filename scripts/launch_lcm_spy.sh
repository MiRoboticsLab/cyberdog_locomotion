#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${DIR}/../common/lcm_type/lcm/java
export CLASSPATH=${DIR}/../common/lcm_type/lcm/java/my_types.jar
pwd

if [ $# == 0 ];then
  com=7667
else
  com=$1
fi

lcm-spy --lcm-url=udpm://239.255.76.67:7669?ttl=255 &
lcm-spy --lcm-url=udpm://239.255.76.67:7670?ttl=255 &
lcm-spy --lcm-url=udpm://239.255.76.67:$com?ttl=255
