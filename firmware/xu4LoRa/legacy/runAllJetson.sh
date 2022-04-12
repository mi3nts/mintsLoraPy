#!/bin/bash
#
sleep 60
kill $(pgrep -f 'python3 nanoReader.py 0')
sleep 10 
python3 nanoReader.py 0 &
sleep 10 
kill $(pgrep -f 'python3 airMarReader.py')
sleep 10 
python3 airMarReader.py &
sleep 5
python3 ipReader.py

