# Automated NCS Benchmark Setup
This repository contains the source code, hardware schematics, 3D printing files, and documentation of the open source benchmark setup described in the paper "Combining Dynamic Deterministic Latency Bounds and Networked Control Systems".

The video below shows a demo of an automated evaluation with 15 configurations and re-swing-ups between each configuration.

[Demo Video](https://github.com/ustutt-ipvs-vs/automated_ncs_benchmark/assets/69897594/da2cac4a-8f67-4f81-817d-bd36dadb8c46)

## Overview
This repository is structured as follows:

- `sender-receiver-linux`: Software components for the sender side and receiver side computers
- `sender-receiver-teensy`: Sender and receiver scripts for the Teensy microcontrollers
- `network-config`: Bash scripts to configure the software switch and the endpoints
    - `scheduling-switch-config`: Scripts to configure the Qdisc and queue sizes at the software switch
- `control-approaches-design`: Files containing calculations for the parameters of the various control approaches for the inverted pendulum.
- `PCB-Design`: Design files of the sender and receiver PCB (KiCad)
- `3d-printable-parts`: Design files of the 3D-printed parts that were used for the construction of the inverted pendulum.
- `photos`: Photos of the inverted pendulum
