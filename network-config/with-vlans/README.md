# Network Configuration Scripts for VLANs:

The devices must be wired like this:
![](ethernet-wiring-setup_29-11-2022.png)

Please note the names of the interfaces.

## Scripts:
Run the scripts on the devices respectively to set up the correct network config.

`configure-switch.sh` does the following
- Sets up VLANs on `enp1s0f0` and `enp1s0f1` with VLAN ID `100`. For this purpose, the VLAN interfaces `enp1s0f0.100` and `enp1s0f1.100` get created.
- Creates a virtual bridge between `enp1s0f0.100` and `enp1s0f1.100`.
- Assigns the IP address to the switch PC.

All other `configure-*.sh` scripts set up a VLAN interface set up VLANs as `eth0.100` interfaces with VLAN ID 100 and assign an IP address.