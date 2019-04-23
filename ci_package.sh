#!/bin/bash

set -e

sudo apt update
sudo apt install libusb-1.0.0-dev libudev-dev -y
./build-all-deb.sh 0.6.11
