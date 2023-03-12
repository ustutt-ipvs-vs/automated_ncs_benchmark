#!/bin/bash

interface=enp1s0f0

# Read queue length from the first argument:
queueLength=$1

# Reduce queue lengths:
bash "$(dirname "$0")/setup-queue-length.sh" $interface $queueLength

# Set up scheduling:
sudo tc qdisc replace dev $interface root handle 1 prio bands 8 priomap 0 1 2 3 4 5 6 7 7 7 7 7 7 7 7 7

# Show results:
tc qdisc show dev $interface
