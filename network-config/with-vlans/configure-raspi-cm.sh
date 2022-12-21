#!/bin/bash

# Add VLAN (with VLAN ID 100)
sudo ip link add link eth0 name eth0.100 type vlan id 100

# Enable all involved interfaces
sudo ip link set dev eth0 up
sudo ip link set dev eth0.100 up

# Set IP address
sudo ip address add 10.0.1.2/24 dev eth0.100
