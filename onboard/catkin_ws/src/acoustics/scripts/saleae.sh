#!/bin/bash

# Start the Saleae Logic software, to be run as root

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

function finish {
  killall Logic
  killall Xvfb
  killall xvfb-run
}

# If we pass in a 1, then we start the Saleae Logic software
if [ $1 == 1 ] 
then
    finish
    xvfb-run Logic --automation --no-sandbox
fi

# If we pass in a 0, then we stop the Saleae Logic software
if [ $1 == 0 ] 
then
    finish
fi

