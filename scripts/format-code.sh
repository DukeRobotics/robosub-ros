#!/bin/bash

# A script that automatically formats code to the standard.

# Run python formatter - Takes a couple of minutes
autopep8 -r --in-place --aggressive --aggressive ../
