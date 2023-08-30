#!/bin/bash
set -x
ping -c 3 192.168.44.233
ssh  -o "StrictHostKeyChecking no" root@192.168.44.233 "ps | grep -E 'manager|mit|imu|cyberdog_control' | grep -v grep | awk '{print \$1}' | xargs kill -9"
ssh  -o "StrictHostKeyChecking no" root@192.168.44.233  'rm -rf /robot/robot-software'
scp -r -o "StrictHostKeyChecking no" /home/mi/robot-software root@192.168.44.233:/robot/
ssh  -o "StrictHostKeyChecking no" root@192.168.44.233 'sync'
ssh  -o "StrictHostKeyChecking no" root@192.168.44.233 "export LD_LIBRARY_PATH=/robot/robot-software/build;/robot/robot-software/manager /robot/ > /robot/manager.log 2>&1 &"
