#!/bin/bash

DIR=$(cd $(dirname $0);pwd)
enxname=$(ifconfig | grep -B 1 -E '192.168.55.100|192.168.44.100' | grep 'flags' | cut -d ':' -f1)

# sudo ifconfig $enxname 192.168.55.3 netmask 255.255.255.0
${DIR}/config_network_lcm.sh -I  $enxname
