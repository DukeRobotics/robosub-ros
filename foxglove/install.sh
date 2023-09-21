extensions=$(find . -type d -maxdepth 1 -name "*extension")

for dir in ${extensions//;/$'\n'}; do
    cd $dir; npm ci; npm install yarn; npm run local-install; cd ..;
done
