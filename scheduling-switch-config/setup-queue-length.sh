#!/bin/bash

# Usage:
# ./setup-queue-length.sh <interface> <ringBufferSize> <queueLength>
#
# where <interface>, <ringBufferSize> and <queueLength> are optional and have the following default values:
# <interface> = enp1s0f0
# <ringBufferSize> = 80
# <queueLength> = 20


# Get interface from the first argument or use enp1s0f0 as default:
interface=${1:-enp1s0f0} # default: enp1s0f0

ringBufferSize=${2:-80}   # in packets (default: 80)
queueLength=${3:-20}      # in packets (default: 20)

# Set ring buffer size to 80 packets
sudo ethtool -G $interface rx $ringBufferSize
sudo ethtool -G $interface tx $ringBufferSize

# Set linux queue length to 20 packets:
sudo ip link set dev $interface txqlen $queueLength
sudo tc qdisc replace dev $interface root noqueue   # required to trigger reconfiguration of the queue

# Show results:
ethtool -g $interface
echo ""
ip link show dev $interface
echo ""
