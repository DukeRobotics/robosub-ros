#!/bin/bash

# print kernel | find Arduinos | get Serial lines | extract serial number from serial line | flip  list | list uniques | list latest 2 
sudo dmesg | grep Arduino -A1 | grep SerialNumber: | sed -n -e 's/^.*Number: \([^ ]*\)/\1/p' | tac | awk '!seen[$0]++' | head -n2

# print kernel | find Arduinos | get port and convert to /dev/ttyACM format | flip list | list uniques | list latest 2
sudo dmesg | grep ttyACM | sed -n 's/^.* \([^ ]*\): USB ACM device/\/dev\/\1/p' | tac | awk '!seen[$0]++' | head -n2
