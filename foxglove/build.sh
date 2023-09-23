#!/bin/bash

# Check if entr is installed
if ! command -v entr &> /dev/null
then
    echo "entr not found, please install it first: https://github.com/eradman/entr"
    exit 1
fi

trap SIGINT SIGTERM

monitor_folder() {
    local folder="$1"
    # Find all files in the given folder and monitor them using entr
    ls $folder/src/* | entr -d sh -c "cd $folder && npm run local-install"
}

# Find all folders matching '*extension' and start monitoring them
extensions=$(find . -type d -maxdepth 1 -name "*extension")
for folder in ${extensions//;/$'\n'}; do
    echo $folder
    monitor_folder "$folder" &
done

while true; do sleep 86400; done