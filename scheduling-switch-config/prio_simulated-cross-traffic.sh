#!/bin/bash

# Usage:
# Option 1: Use predefined values for low, medium, or high latency:
# ./prio_simulated-cross-traffic.sh [ls|lm|lh]
#
# Option 2: Use custom values for the 8 priority classes:
# ./prio_simulated-cross-traffic.sh [delay0] [delay1] [delay2] [delay3] [delay4] [delay5] [delay6] [delay7]

interface=enp1s0f0
netemQueueLength=20 # in packets

# Values are in milliseconds:
low_latency_values=(2 4 6 8 10 12 14 16)
medium_latency_values=(14 16 18 20 22 24 26 28)	# Original: (5 10 15 20 25 30 35 40)
high_latency_values=(10 20 30 40 50 60 70 80)

# If first argument is 'ls' use low latency values, if it is 'lm' use medium latency values, 
# if it is 'lh' use high latency values, otherwise use the given values:
if [ "$1" == "ls" ]; then
    delays=("${low_latency_values[@]}")
elif [ "$1" == "lm" ]; then
    delays=("${medium_latency_values[@]}")
elif [ "$1" == "lh" ]; then
    delays=("${high_latency_values[@]}")
else
    delays=("$@")
    # Abort if not exactly 8 values are given:
    if [ ${#delays[@]} -ne 8 ]; then
        echo "Error: Exactly 8 values must be given as arguments."
        exit 1
    fi
fi

# Print delay values:
echo "Delays for priority classes 0 to 7 in milliseconds:"
echo ${delays[@]}


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
