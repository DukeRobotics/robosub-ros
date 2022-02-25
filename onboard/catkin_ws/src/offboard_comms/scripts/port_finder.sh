#!/bin/sh

sudo dmesg | \
grep tty | \
sed -n -e 's/^.* \([^ ]*\): USB ACM device/\1/p' | \
tail -n 1 | \
sed -n -e 's/^/\/dev\//p' | \
tr -d '\n' | \
cat
