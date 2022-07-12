#!/bin/sh

while :
do
    sudo python3 dev_lowlevel_loopback.py
    #echo $(date | cut -c 55-69)
    echo $(date)
done
