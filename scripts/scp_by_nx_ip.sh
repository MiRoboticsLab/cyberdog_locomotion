#!/bin/bash
set -x

if [[ $# != 1 ]]; then
    echo "Please input the NX IP"
    exit
fi
nx_ip=$1
sshpass -p '123' ssh -o "StrictHostKeyChecking no" mi@${nx_ip} 'rm -rf /home/mi/robot-software'
sshpass -p '123' scp -r -o "StrictHostKeyChecking no" ../onboard-build/robot-software mi@${nx_ip}:/home/mi/
sshpass -p '123' scp -r -o "StrictHostKeyChecking no" nx2loco.sh mi@${nx_ip}:/home/mi/robot-software/
sshpass -p '123' ssh -o "StrictHostKeyChecking no" mi@${nx_ip} 'sync'
sshpass -p '123' ssh -o "StrictHostKeyChecking no" mi@${nx_ip} 'cd /home/mi/robot-software;chmod +x nx2loco.sh;./nx2loco.sh'
sshpass -p '123' ssh -o "StrictHostKeyChecking no" mi@${nx_ip} 'rm -rf /home/mi/robot-software'