#!/bin/bash

../scripts/pack_runnable.sh

adb shell rm -r /mnt/UDISK/robot-software
adb push robot-software /mnt/UDISK

adb shell rm -f /mnt/UDISK/manager
adb shell rm -rf /mnt/UDISK/manager_config
adb push ../scripts/manager /mnt/UDISK

adb shell "sync"
