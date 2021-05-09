#!/bin/sh

sudo dmesg | \
grep tty | \
sed -n -e 's/^.* ch341-uart converter now attached to \([^ ]*\).*/\1/p' | \
sed -n -e 's/^/\/dev\//p' | \
tr -d '\n' | \
cat
