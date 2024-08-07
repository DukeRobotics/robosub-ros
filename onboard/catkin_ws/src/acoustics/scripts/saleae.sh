#!/bin/bash

# Start the Saleae Logic software, to be run as root

# if [ "$EUID" -ne 0 ]
#   then echo "Please run as root"
#   exit
# fi

function finish {
  killall Logic
  killall Xvfb
  killall xvfb-run
}

if [ $1 == 'enable' ] 
then
    finish
    xvfb-run Logic --automation --no-sandbox
fi

if [ $1 == 'disable' ]
then
    finish
fi

