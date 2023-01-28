#!/bin/bash

interface=enp1s0f0
netemQueueLength=20 # in packets
delays=(0 5 10 15 20 25 30 35)  # delays for priority classes 0 to 7 in milliseconds

# Reduce queue lengths:
bash "$(dirname "$0")/setup-queue-length.sh" $interface

# Set up priority scheduling with 8 priority classes:
sudo tc qdisc replace dev $interface root handle 1 prio bands 8 priomap 0 1 2 3 4 5 6 7 7 7 7 7 7 7 7 7

# Add an individual delay to each priority class:
sudo tc qdisc replace dev $interface parent 1:1 handle 21 netem delay ${delays[0]}ms limit $netemQueueLength
sudo tc qdisc replace dev $interface parent 1:2 handle 22 netem delay ${delays[1]}ms limit $netemQueueLength
sudo tc qdisc replace dev $interface parent 1:3 handle 23 netem delay ${delays[2]}ms limit $netemQueueLength
sudo tc qdisc replace dev $interface parent 1:4 handle 24 netem delay ${delays[3]}ms limit $netemQueueLength
sudo tc qdisc replace dev $interface parent 1:5 handle 25 netem delay ${delays[4]}ms limit $netemQueueLength
sudo tc qdisc replace dev $interface parent 1:6 handle 26 netem delay ${delays[5]}ms limit $netemQueueLength
sudo tc qdisc replace dev $interface parent 1:7 handle 27 netem delay ${delays[6]}ms limit $netemQueueLength
sudo tc qdisc replace dev $interface parent 1:8 handle 28 netem delay ${delays[7]}ms limit $netemQueueLength

# Show results:
tc qdisc show dev $interface
