#
import serial
import datetime
from mintsXU4 import mintsSensorReader as mSR
from mintsXU4 import mintsDefinitions as mD
import sys
import time

dataFolder  = mD.dataFolder
nanoPorts   = mD.nanoPorts
baudRate    = 9600

def main(portNum):
    if(len(nanoPorts)>0):

        ser = serial.Serial(
        port= nanoPorts[portNum],\
        baudrate=baudRate,\
        parity  =serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
        timeout=0)

        print(" ")
        print("Connected to: " + ser.portstr)
        print(" ")

        #this will store the line
        line = []

        startTime = time.time()
        while True:
            try:
                for c in ser.read():
                    line.append(chr(c))

                    if chr(c) == '~':
                        startTime = time.time()
                        # print(line)
                        dataString     = (''.join(line))
                        dataStringPost = dataString.replace('~', '')
                        print("================")
                        print(dataStringPost)
                        mSR.dataSplit(dataStringPost,datetime.datetime.now())
                        line = []
                        break

                if time.time()-startTime> 10:
                    print("No Data Returned, Closing Serial Port")
                    print("Serial Port Closed")
                    ser.close
                    time.sleep(5)
                    radCheck(portNum)
                    
            except:
                print("Incomplete String Read")
                line = []
        ser.close()




def radCheck(portNum):
    if(len(nanoPorts)>0):

        ser = serial.Serial(
        port= nanoPorts[portNum],\
        baudrate=115200,\
        parity  =serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
        timeout=0)

        print(" ")
        print("Connected to: " + ser.portstr)
        print(" ")

        #this will store the line
        line = []

        startTime = time.time()
        while True:
            try:
                for c in ser.read():
                    line.append(chr(c))

                    if chr(c) == '\n':
                        startTime = time.time()
                        # print(line)
                        dataString     = (''.join(line))
                        dataStringPost = dataString.replace('\n', '')
                        currentDateTime = datetime.datetime.now()
                        if dataString.find('START Watchdog Reset;')>0:
                            dataStringPost = dataStringPost.replace('START Watchdog Reset;', '')
                            mSR.QLMRAD001Write(dataStringPost,currentDateTime)
                            mSR.QLMRAD001Write("-100",currentDateTime)
                        else:
                            print("================")
                            print(dataStringPost)
                            mSR.QLMRAD001Write(dataStringPost,currentDateTime)
                        line = []
                        break

                if time.time()-startTime> 120:

                    print("No Data Returned, Closing Serial Port")
                    ser.close
                    time.sleep(5)
                    sys.exit()
                    
            except:
                print("Incomplete String Read")
                line = []
        ser.close()





if __name__ == "__main__":
    print("=============")
    print("    MINTS    ")
    print("=============")
    portNum = int(sys.argv[1])
    print("Number of Arduino Nano devices: {0}".format(len(nanoPorts)))
    print("Monitoring Arduino Nano on port: {0}".format(nanoPorts[portNum]) + " with baudrate " + str(baudRate))
    main(portNum)



## OLD CODE
# import serial
# import datetime
# from mintsXU4 import mintsSensorReader as mSR
# from mintsXU4 import mintsDefinitions as mD
#
# dataFolder  = mD.dataFolder
# nanoPorts    = mD.nanoPorts
#
# def main():
#     if(len(nanoPorts)>1):
#
#         ser = serial.Serial(
#         port= nanoPorts[1],\
#         baudrate=9600,\
#         parity  =serial.PARITY_NONE,\
#         stopbits=serial.STOPBITS_ONE,\
#         bytesize=serial.EIGHTBITS,\
#         timeout=0)
#
#         print("connected to: " + ser.portstr)
#
#         #this will store the line
#         line = []
#
#         while True:
#             try:
#                 for c in ser.read():
#                     line.append(chr(c))
#                     if chr(c) == '\n': # line ends at newline character
#                     	dataString = ''.join(line)
#                         dataStringPost = dataString.replace('\n', '')
#                         print(dataStringPost)
#                         mSR.dataSplit(dataStringPost,datetime.datetime.now())
#                         line = []
#                         break
#             except:
#                 print("Incomplete String Read")
#                 line = []
#         ser.close()
#
#
# if __name__ == "__main__":
#    main()
