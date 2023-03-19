#!/bin/bash

# Add VLAN (with VLAN ID 100)
# The ingress-qos-map maps the VLAN PCP to the skb->priority.
# The egress-qos-map maps the skb->priority to the VLAN PCP.
sudo ip link add link eth0 name eth0.100 type vlan id 100 ingress-qos-map 0:0 1:1 2:2 3:3 4:4 5:5 6:6 7:7 egress-qos-map 0:0 1:1 2:2 3:3 4:4 5:5 6:6 7:7

# Enable all involved interfaces
sudo ip link set dev eth0 up
sudo ip link set dev eth0.100 up

# Set IP address
sudo ip address add 10.0.1.5/24 dev eth0.100
