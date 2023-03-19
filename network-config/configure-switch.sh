#!/bin/bash

# Setup interfaces for VLANs (on VLAN ID 100):
# The ingress-qos-map maps the VLAN PCP to the skb->priority.
# The egress-qos-map maps the skb->priority to the VLAN PCP.
sudo ip link add link enp1s0f0 name enp1s0f0.100 type vlan id 100 ingress-qos-map 0:0 1:1 2:2 3:3 4:4 5:5 6:6 7:7 egress-qos-map 0:0 1:1 2:2 3:3 4:4 5:5 6:6 7:7
sudo ip link add link enp1s0f1 name enp1s0f1.100 type vlan id 100 ingress-qos-map 0:0 1:1 2:2 3:3 4:4 5:5 6:6 7:7 egress-qos-map 0:0 1:1 2:2 3:3 4:4 5:5 6:6 7:7
sudo ip link add link enp1s0f3 name enp1s0f3.100 type vlan id 100 ingress-qos-map 0:0 1:1 2:2 3:3 4:4 5:5 6:6 7:7 egress-qos-map 0:0 1:1 2:2 3:3 4:4 5:5 6:6 7:7

# Create bridge
sudo ip link add name br0 type bridge
sudo ip link set dev enp1s0f0.100 master br0
sudo ip link set dev enp1s0f1.100 master br0
sudo ip link set dev enp1s0f3.100 master br0

# Enable promiscious mode
sudo ip link set dev enp1s0f0.100 promisc on
sudo ip link set dev enp1s0f1.100 promisc on
sudo ip link set dev enp1s0f3.100 promisc on

# Turn on all involved interfaces:
sudo ip link set dev enp1s0f0.100 up
sudo ip link set dev enp1s0f1.100 up
sudo ip link set dev enp1s0f3.100 up
sudo ip link set dev enp1s0f0 up
sudo ip link set dev enp1s0f1 up
sudo ip link set dev enp1s0f3 up
sudo ip link set dev br0 up

# Set IP address of switch pc:
sudo ip address add 10.0.1.4/24 dev br0


# The custom scheduling then needs to be applied to enp1s0f0 (NOT enp1s0f0.100).
# The priority provided through the VLAN PCP can be read from the skb->priority.
