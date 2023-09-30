#!/bin/bash
# A convenience script to automatically install all Foxglove extensions.
# Foxglove extensions are defined as any folder matching '*-panel' in the current directory.
# Usage: ./install.sh (Must be run from the root of the Foxglove directory)

if [[ $cwd_name != "foxglove" ]]; then
    echo "./install must be run inside the 'foxglove' directory"
    exit 1
fi

extensions=$(find . -type d -maxdepth 1 -name "*panel")

for dir in ${extensions//;/$'\n'}; do
    (
        cd "$dir" || exit
        npm install
        npm install yarn
        yarn install
        npm run local-install
        rm yarn.lock  # only want package-lock.json but to run local-install we need to run `yarn install` first
    )
done
