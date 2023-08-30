#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${DIR}/../common/lcm_type/lcm/java
export CLASSPATH=${DIR}/../common/lcm_type/lcm/java/my_types.jar
pwd
lcm-logger
