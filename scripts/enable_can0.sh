#!/bin/bash

set -e

sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ifconfig can0 txqueuelen 100

