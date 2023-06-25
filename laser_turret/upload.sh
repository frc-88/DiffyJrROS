#!/bin/bash

DEVICE=${1:-/dev/ttyACM0}

ampy -p $DEVICE put main.py
