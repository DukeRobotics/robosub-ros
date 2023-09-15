#!/bin/bash

extension_dirs=$(find . -type d -maxdepth 1 -name "*extension")
# args=()
# arr=()
# for dir in ${extension_dirs//;/$'\n'}; do
#     args+=("$dir"/src/"*")
#     arr+=( "$(ls $dir/src/)" )
# done

# var=$( IFS=$' '; echo "${args[*]}" )
# echo "var $var"

# pwd

# arr=$( IFS=$' '; echo "${arr[*]}" )
# echo "arr $arr"

# while true; do echo $arr | entr -pd 
# for dir in ${extension_dirs//;/$'\n'}; do
#     cd "$dir/"; pwd; npm run local-install; cd ..
# done

# echo $( "ls $var" )

# var=$( ls */src/* )
# echo $var

while true; do ls */src/* | entr -pd 
for dir in ${extension_dirs//;/$'\n'}; do
    echo "detected"; cd $dir; npm run local-install; cd ..;
done
done

# while true; do ls ./call-service-panel-extension/src/* ./custom-image-extension/src/* ./publish-topic-extension/src/* |
#     entr -pd echo "detected stuff"; cd call-service-panel-extension; npm run local-install; cd ..;
#              cd custom-image-extension; npm run local-install; cd ..;
#              cd publish-topic-extension; npm run local-install; cd ..;
#     done
