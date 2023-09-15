#!/bin/bash

extensions=$(find . -type d -maxdepth 1 -name "*extension")

function build {
    for dir in ${extensions//;/$'\n'}; do
        cd $dir; npm run local-install; cd ..;
    done
}

while true; do ls *-extension/src/* | entr -pd -s 'kill $PPID'; build; done
