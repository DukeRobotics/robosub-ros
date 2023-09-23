#!/bin/bash
# A convenience script to automatically install all Foxglove extensions.
# Foxglove extensions are defined as any folder matching '*-extension' in the current directory.
# Usage: ./install.sh (Must be run from the root of the Foxglove directory)

extensions=$(find . -type d -maxdepth 1 -name "*extension")

for dir in ${extensions//;/$'\n'}; do
    cd $dir; npm ci; npm run local-install; cd ..;
done
