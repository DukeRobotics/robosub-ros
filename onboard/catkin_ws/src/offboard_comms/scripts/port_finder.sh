#!/bin/sh

sudo dmesg | \
grep "USB ACM device" | \
awk '{print $5;}' | \
sed 's/.$//' | \
sed -n -e 's/^/\/dev\//p' | \
tr -d '\n' | \
cat
