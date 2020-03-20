#!/bin/bash

sudo rmmod pcan

sudo sh -c "echo 0 > /sys/bus/usb/devices/$1/authorized"
sudo sh -c "echo 1 > /sys/bus/usb/devices/$1/authorized"

