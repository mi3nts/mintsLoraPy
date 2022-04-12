#!/bin/bash
#
sleep 60
python3 nanoReader.py 0 &
sleep 5
python3 nanoReader.py 1 &
sleep 5
python3 nanoReader.py 2 &
sleep 5
python3 ips7100ReaderV1.py & 
sleep 5 
python3 ozoneReader.py &  
sleep 5 
python3 GPSReader.py &
sleep 5
python3 ipReader.py
sleep 5

# python3 centralNodeReaderNano.py &
# sleep 10
# cd odroidShow2 && python3 mintsShow2.py
