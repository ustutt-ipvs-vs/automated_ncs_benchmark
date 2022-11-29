#!/bin/bash

# Run this command on the computer that should be used as the switch.

# Rename existing netplan configs:
for file in /etc/netplan/*yaml
do
    sudo mv "$file" "$file.bak"
done

# Copy netplan config to config directory:
sudo cp 01-switch-netplan-config.yaml /etc/netplan/.

# Enable netplan config:
sudo netplan generate
sudo netplan apply

# Enable Proxy ARP:
echo 1 | sudo tee /proc/sys/net/ipv4/conf/all/proxy_arp
echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward
