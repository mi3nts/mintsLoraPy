# MQTT Client demo
# Continuously monitor two different MQTT topics for data,
# check if the received data matches two predefined 'commands'
import base64
from cgitb import strong
from this import d
import paho.mqtt.client as mqtt
import datetime
import yaml
import collections
import json
import time 
import serial.tools.list_ports
from collections import OrderedDict

from mintsXU4 import mintsSensorReader as mSR
from mintsXU4 import mintsDefinitions as mD
from mintsXU4 import mintsLatest as mL
from mintsXU4 import mintsLoRaReader as mLR
from mintsXU4 import mintsLiveNodes as mLN
from collections import OrderedDict
import struct
import numpy as np

loRaE5MiniPorts     = mD.loRaE5MiniPorts
canareePorts        = mD.canareePorts
gpsPorts            = mD.gpsPorts
appKey              = mD.keys['keys']['appKey']
macAddress          = mD.macAddress

def openSerial(portIn,baudRate):
    ser = serial.Serial(
            port= portIn,\
            baudrate=baudRate,\
            parity  =serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
            timeout=0)

    print(" ")
    print("Connected to: " + ser.portstr)
    print(" ")
    return ser;

def sendCommand2(serIn,commandStrIn,timeOutIn):
    serIn.write(str.encode(commandStrIn+ '\n\r'))
    line = []
    lines = []
    startTime = time.time()
    while (time.time()-startTime)<timeOutIn:
        for c in serIn.read():
            line.append(chr(c))
            if chr(c) == '\n':
                dataString = (''.join(line)).replace("\n","").replace("\r","")
                lines.append(dataString)
                print(dataString)
                line = []
                break
    return serIn,lines;


def sendCommand(serIn,commandStrIn,timeOutIn):
    serIn.write(str.encode(commandStrIn+ '\n\r'))
    line = []
    lines = []
    startTime = time.time()
    while (time.time()-startTime)<timeOutIn:
        for c in serIn.read():
            line.append(chr(c))
            if chr(c) == '\n':
                dataString = (''.join(line)).replace("\n","").replace("\r","")
                lines.append(dataString)
                print(dataString)
                line = []
                break
    return lines;

def readSerialLine(serIn,timeOutSensor,sizeExpected):
    line = []
    startTime = time.time()
    startFound = False
    while (time.time()-startTime)<timeOutSensor:   
        # try:
            for c in serIn.read():
                line.append(chr(c))
                # print((''.join(line)))

                if chr(c) == '\n':
                    if startFound == True:
                        dataString     = (''.join(line))
                        dataStringPost = dataString.replace('\r\n', '')
                        dataStringData =  dataStringPost.split(',')
                        if sizeExpected == len(dataStringData):
                            return dataStringData;
                        else:
                            line = []
                    else:    
                        startFound = True
                        line = []

        # except:
        #     print("Incomplete String Read")
        #     line = []

def readSerialLineStr(serIn,timeOutSensor,strExpected):
    line = []
    startTime = time.time()
    startFound = False
    while (time.time()-startTime)<timeOutSensor:   
        # try:
            for c in serIn.read():
                line.append(chr(c))
                # print((''.join(line)))

                if chr(c) == '\n':
                    if startFound == True:
                        dataString     = (''.join(line))
                        dataStringPost = dataString.replace('\r\n', '')
                        dataStringData =  dataStringPost.split(',')
                        
                        if dataStringPost.find(strExpected) >0:
                            return dataStringData;
                        else:
                            line = []
                    else:    
                        startFound = True
                        line = []

        # except:
        #     print("Incomplete String Read")
        #     line = []




def swapBytes(inputIn):
    return bytes([c for t in zip(inputIn[1::2], inputIn[::2]) for c in t])


def getMessegeStringHex(dataIn, sensorIn):
     if sensorIn == "IPS7100CNR":
        strOut  = \
            np.uint32(dataIn[1]).tobytes().hex().zfill(8)+ \
            np.uint32(dataIn[3]).tobytes().hex().zfill(8) + \
            np.uint32(dataIn[5]).tobytes().hex().zfill(8)+ \
            np.uint32(dataIn[7]).tobytes().hex().zfill(8) + \
            np.uint32(dataIn[9]).tobytes().hex().zfill(8)+ \
            np.uint32(dataIn[11]).tobytes().hex().zfill(8) + \
            np.uint32(dataIn[13]).tobytes().hex().zfill(8)+ \
            np.float32(dataIn[15]).tobytes().hex().zfill(8)+ \
            np.float32(dataIn[17]).tobytes().hex().zfill(8) + \
            np.float32(dataIn[19]).tobytes().hex().zfill(8)+ \
            np.float32(dataIn[21]).tobytes().hex().zfill(8) + \
            np.float32(dataIn[23]).tobytes().hex().zfill(8)+ \
            np.float32(dataIn[25]).tobytes().hex().zfill(8) + \
            np.float32(dataIn[27]).tobytes().hex().zfill(8)

        return strOut;

     if sensorIn == "BME688CNR":
        strOut  = \
            np.float32(dataIn[29]).tobytes().hex().zfill(8)+ \
            np.float32(dataIn[31]).tobytes().hex().zfill(8) + \
            np.float32(dataIn[33]).tobytes().hex().zfill(8)+ \
            np.float32(dataIn[35]).tobytes().hex().zfill(8) + \
            np.float32(dataIn[37]).tobytes().hex().zfill(8)+ \
            np.float32(dataIn[39]).tobytes().hex().zfill(8) + \
            np.float32(dataIn[41]).tobytes().hex().zfill(8)
        return strOut;

    
def joinNetwork(numberOfTries,ser,timeOutIn):
    for currentTry in range(numberOfTries):
        print("Joining Network Trial: " + str(currentTry))
        lines = sendCommand(ser,'AT+JOIN',timeOutIn)
        
        for line in lines:
            if line == '+JOIN: Network joined':
                return True;

    return False;
    






if __name__ == "__main__":

    # To make sure everything is working run python3 mintsDefinitions.py 
    
    # print("Mac Address          : {0}".format(macAddress))
    # print("LoRa E5 Mini Ports :")
    # for dev in loRaE5MiniPorts:
    #     print("\t{0}".format(dev))
    
    # print("Canaree Ports:")
    # for dev in canareePorts:
    #     print("\t{0}".format(dev))
    
    # print("GPS Ports:")
    # for dev in gpsPorts:
    #     print("\t{0}".format(dev))

    # Establishing connection to the serial port
    if(len(loRaE5MiniPorts)>0):
        serE5Mini    = openSerial(loRaE5MiniPorts[0],9600)
        serCanaree   = openSerial(canareePorts[0],115200)
        serGPS       = openSerial(gpsPorts[0],9600)
        
        sendCommand(serE5Mini,'AT+RESET',2)
        sendCommand(serE5Mini,'AT+FDEFAULT',1)
        sendCommand(serE5Mini,'AT+VER',1)
        sendCommand(serE5Mini,'AT+FDEFAULT',1)
        sendCommand(serE5Mini,'AT+ID',1)
        sendCommand(serE5Mini,'AT+KEY=APPKEY, "'+appKey+'"',1)
        sendCommand(serE5Mini,'AT+MODE=LWOTAA',1)
        sendCommand(serE5Mini,'AT+DR=US915',1)
        sendCommand(serE5Mini,'AT+DR=dr2',1)
        sendCommand(serE5Mini,'AT+CH=NUM, 56-63',1)
        sendCommand(serE5Mini,'AT+POWER=20',1)
        sendCommand(serE5Mini,'AT+PORT=2',2)
        
        # Check Join
        joined = False 
        joined = joinNetwork(10,serE5Mini,10)

        if not joined:
            time.sleep(60)
            joined  = joinNetwork(10,serE5Mini,10)

        if not joined:
            print("No Network Found")
            quit()
        else:
            print("Network Found")

        messege    = hex(struct.unpack('<I', struct.pack('<I', 254))[0])
        messege = messege.replace('0x','').zfill(2)
        sendCommand(serE5Mini,'AT+MSGHEX='+str(messege),5)


        while True:
            # At this point we work on the canaree 
            sensorData = readSerialLineStr(serGPS,2,"GGA")
            print(sensorData)

            sensorData = readSerialLineStr(serGPS,2,"RMC")
            print(sensorData)

            sensorData = readSerialLine(serCanaree,2,44)
            strOut = getMessegeStringHex(sensorData, "IPS7100CNR")
            sendCommand(serE5Mini,'AT+PORT=17',2)
            sendCommand(serE5Mini,'AT+MSGHEX='+str(strOut),5)


            sensorData = readSerialLine(serCanaree,2,44)
            strOut = getMessegeStringHex(sensorData, "BME688CNR")
            sendCommand(serE5Mini,'AT+PORT=25',2)
            sendCommand(serE5Mini,'AT+MSGHEX='+str(strOut),5)




















