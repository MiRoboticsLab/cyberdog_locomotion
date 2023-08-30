#!/bin/bash
set -x -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd ${DIR}/../onboard-build/
rm -rf robot-software
mkdir robot-software
mkdir robot-software/build
mkdir robot-software/common
mkdir -p robot-software/control/motion_list
mkdir -p robot-software/control/rl_models
mkdir -p robot-software/control/rl_models/cyberdog2
mkdir -p robot-software/control/offline_trajectory

cp version.txt robot-software/
cp control/user/cyberdog_control robot-software/build/
cp hardware/manager robot-software/

cp -r ../control/motion_list/ ./robot-software/control/
cp -r ../control/rl_models/ ./robot-software/control/
cp -r ../control/offline_trajectory ./robot-software/control/
cp -r ../common/config ./robot-software/common
cp -r ../hardware/json/*.json ./robot-software/common/config/

find . -name \*.so* -exec cp {} ./robot-software/build \;

if [ -d "../factory" ]; then
  mkdir robot-software/factory
  cp -r ../factory/json/*.json ./robot-software/common/config/
  find ./factory -name \*_test -exec cp {} ./robot-software/factory \;
  find . -name \*.so* -exec cp {} ./robot-software/factory \;
fi