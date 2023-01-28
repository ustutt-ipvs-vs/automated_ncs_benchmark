#!/bin/bash

interface=enp1s0f0

# Reduce queue lengths:
bash setup-queue-length.sh $interface

# Set up scheduling:
sudo tc qdisc replace dev $interface root handle 1 prio bands 8 priomap 0 1 2 3 4 5 6 7 7 7 7 7 7 7 7 7

# Show results:
tc qdisc show dev $interface
