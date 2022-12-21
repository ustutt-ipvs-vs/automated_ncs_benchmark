#!/bin/bash

# Setup interfaces for VLANs (on VLAN ID 100)
sudo ip link add link enp1s0f0 name enp1s0f0.100 type vlan id 100
sudo ip link add link enp1s0f1 name enp1s0f1.100 type vlan id 100

# Create bridge
sudo ip link add name br0 type bridge
sudo ip link set dev enp1s0f0.100 master br0
sudo ip link set dev enp1s0f1.100 master br0

# Enable promiscious mode
sudo ip link set dev enp1s0f0.100 promisc on
sudo ip link set dev enp1s0f1.100 promisc on

# Turn on all involved interfaces:
sudo ip link set dev enp1s0f0.100 up
sudo ip link set dev enp1s0f1.100 up
sudo ip link set dev enp1s0f0 up
sudo ip link set dev enp1s0f1 up
sudo ip link set dev br0 up

# Set IP address of switch pc:
sudo ip address add 10.0.1.4/24 dev br0

