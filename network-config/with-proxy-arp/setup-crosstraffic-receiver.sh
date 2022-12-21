#!/bin/bash

# Run this command on the computer that should be used as the receiver.

# Rename existing netplan configs:
for file in /etc/netplan/*yaml
do
    sudo mv "$file" "$file.bak"
done

# Copy netplan config to config directory:
sudo cp 01-crosstraffic-receiver-netplan-config.yaml /etc/netplan/.

# Enable netplan config:
sudo netplan generate
sudo netplan apply
