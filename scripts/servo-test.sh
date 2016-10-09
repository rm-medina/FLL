#!/bin/bash

read -p "Enter channel1   [5   ]   " channel1
read -p "Enter channel2   [2   ]   " channel2

read -p "Enter device     [0   ]   " device
read -p "Enter start pos  [4000]   " start
read -p "Enter end pos    [8000]   " end

while [ 1 ];  do
    ../servolib/test-servo --dev $device --channel $channel1 --write --pos $start
    ../servolib/test-servo --dev $device --channel $channel2 --write --pos $start
    sleep 1
    ../servolib/test-servo --dev $device --channel $channel1 --write --pos $end
    ../servolib/test-servo --dev $device --channel $channel2 --write --pos $end   
    sleep 1
done



