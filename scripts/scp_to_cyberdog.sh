#!/bin/bash
set -x
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
${DIR}/../scripts/pack_runnable.sh

cyberdog2_ip='192.168.44.233'
cyberdog_ip='192.168.55.233'
if ping -c 1 -w 3 ${cyberdog2_ip} > /dev/null;then
	echo "ping cyberdog2 ok, ready for flash!"
	ip=${cyberdog2_ip}
elif ping -c 1 -w 3 ${cyberdog_ip} > /dev/null;then
	echo "ping cyberdog ok, ready for flash!"
	ip=${cyberdog_ip}
else
	echo "Communication failed, please check the network config!"
	exit
fi

ssh root@${ip} "ps | grep -E 'manager|mit|imu|cyberdog_control' | grep -v grep | awk '{print \$1}' | xargs kill -9"

if scp root@${ip}:/mnt/UDISK/test/manager.sh ./ >&/dev/null ; then 
	echo "copy to /mnt/UDISK !";
	ssh root@${ip}  'rm -rf /mnt/UDISK/robot-software'
	scp -r ../onboard-build/robot-software root@${ip}:/mnt/UDISK/
	ssh root@${ip} 'sync'
	# overwrite cyberdog old manager
	ssh root@${ip}  'rm -rf /mnt/UDISK/manager'
	ssh root@${ip}  'rm -rf /mnt/UDISK/manager_config'
	scp ../scripts/manager root@${ip}:/mnt/UDISK/manager
	ssh root@${ip} 'sync'

	ssh root@${ip} "export LD_LIBRARY_PATH=/mnt/UDISK/robot-software/build;/mnt/UDISK/robot-software/manager /mnt/UDISK/ > /mnt/UDISK/manager_log/manager.log 2>&1 &"
else
	echo "copy to /robot !" ;
	ssh root@${ip}  'rm -rf /robot/robot-software'
	scp -r ../onboard-build/robot-software root@${ip}:/robot/
	ssh root@${ip} 'sync'
	ssh root@${ip} "export LD_LIBRARY_PATH=/robot/robot-software/build;/robot/robot-software/manager /robot/ > /robot/manager.log 2>&1 &"
fi