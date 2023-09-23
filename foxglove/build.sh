#!/bin/bash
# A convenience script used to automatically rebuild Foxglove extensions when changes are detected.
# Foxglove extensions are defined as any folder matching '*-extension' in the current directory.
# Only the 'src' folder is monitored for changes.
# NOTE: This script requires 'entr' to be installed: https://github.com/eradman/entr
# Usage: ./build.sh (Must be run from the root of the Foxglove directory)

# Check if entr is installed
if ! command -v entr &> /dev/null
then
    echo "'entr' could not be found, please install it: https://github.com/eradman/entr"
    exit 1
fi

# Kill all child processes on exit
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

# Monitor a package for src changes and run 'npm run local-install' when a change is detected
continuous_build() {
    local folder="$1"
    ls $folder/src/* | entr -d sh -c "cd $folder && npm run local-install"
}

# Find all folders matching '*-extension' and start child processes to monitor them
extensions=$(find . -type d -maxdepth 1 -name "*-extension")
for folder in ${extensions//;/$'\n'}; do
    echo "Monitoring $folder"
    continuous_build "$folder" &
done

# Block until signal
while :; do sleep 86400; done