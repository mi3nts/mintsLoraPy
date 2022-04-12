from datetime import datetime
from os import name
import time
from collections import deque
from mintsXU4 import mintsSensorReader as mSR
from mintsXU4 import mintsDefinitions as mD
from mintsXU4 import mintsLatest as mL
import numpy as np
from datetime import datetime, timedelta
from time import mktime


dataFolder          = mD.dataFolder
ipsPort             = mD.ipsPorts[0]
baudRate            = 9600
ipsFrequency        = mD.ipsFrequency


class ips:
    def __init__(self,port,baudRate,frequency ):
    
        self.lookBack            = timedelta(seconds=frequency) 
         
        ## Init 
        self.initRun = True

        ## IPS Data
        self.pc0_1           = []
        self.pc0_3           = []
        self.pc0_5           = []
        self.pc1_0           = []
        self.pc2_5           = []
        self.pc5_0           = []
        self.pc10_0          = []
        self.pm0_1           = []
        self.pm0_3           = []
        self.pm0_5           = []
        self.pm1_0           = []
        self.pm2_5           = []
        self.pm5_0           = []
        self.pm10_0          = []
        self.dateTime        = []
        ser = serial.Serial(
        port= port,\
        baudrate=baudRate,\
        parity  =serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
        timeout=0)
        print(" ")
        print("Connected to: " + ser.portstr)
        print(" ")
  
        while True:
            try:
                for c in ser.read():
                    line.append(chr(c))
                    if chr(c) == '\n':
                        dataString     = (''.join(line))
                        dataStringPost = dataString.replace('\n', '')
                        print("================")
                        print(dataStringPost)
                        self.reader(dataStringPost,datetime.datetime.now())
                        line = []
                        break
            except:
                print("Incomplete String Read")
                line = []
        ser.close()  
  
  
      def poper(self):

        if self.dateTime[0] < self.currentTime - self.lookBack  :
            self.pm1.pop(0) #remove oldest
            self.pm2_5.pop(0) #remove oldest
            self.pm4.pop(0) #remove oldest
            self.pm10.pop(0) #remove oldest
            self.dateTime.pop(0) #remove oldest 
            if len(self.dateTime) == 0:
                self.initRun = True
            else:
                self.recursivePoper()
        else:
            return 
        

    def updater(self):
        self.pc0_1.append(self.pc0_1_Now)
        self.pc0_3.append(self.pc0_3_Now)
        self.pc0_5.append(self.pc0_5_Now)
        self.pc1_0.append(self.pc1_0_Now)  
        self.pc2_5.append(self.pc2_5_Now)
        self.pc5_0.append(self.pc5_0_Now)  
        self.pc10_0.append(self.pc10_0_Now)        
        self.pm0_1.append(self.pc0_1_Now)
        self.pm0_3.append(self.pc0_3_Now)
        self.pm0_5.append(self.pc0_5Now)
        self.pm1_0.append(self.pc1_0_Now)  
        self.pm2_5.append(self.pc2_5_Now)
        self.pm5_0.append(self.pc5_0_Now)  
        self.pm10_0.append(self.pc10_0_Now)   
 
    def reader(self):

        dataIn = mL.readJSONLatestAllMQTT("0001c0231d43","FRG001")[0]
        self.ctNow =  datetime.strptime(dataIn['dateTime'],'%Y-%m-%d %H:%M:%S').replace(tzinfo=tz.tzutc()).astimezone(tz.gettz())
        self.poper()
        self.updater()

                    

if __name__ == '__main__':
    print("=============")
    print("    MINTS    ")
    print("=============")

    i = ips(ipsPort,baudRate,ipsFrequency)
 
