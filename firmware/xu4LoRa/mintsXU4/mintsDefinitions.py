
# from turtle import st
from getmac import get_mac_address
import serial.tools.list_ports
import datetime 
import time 
import yaml

def findPorts(strIn1,strIn2):
    ports = list(serial.tools.list_ports.comports())
    allPorts = []
    for p in ports:
        currentPortStr1 = str(p[1])
        currentPortStr2 = str(p[2])
        if(currentPortStr1.find(strIn1)>=0 and currentPortStr2.find(strIn2)>=0 ):
            allPorts.append(str(p[0]).split(" ")[0])
    return allPorts

def findMacAddress():
    macAddress= get_mac_address(interface="eth0")
    if (macAddress!= None):
        return macAddress.replace(":","")

    macAddress= get_mac_address(interface="docker0")
    if (macAddress!= None):
        return macAddress.replace(":","")

    macAddress= get_mac_address(interface="enp1s0")
    if (macAddress!= None):
        return macAddress.replace(":","")

    macAddress= get_mac_address(interface="enp31s0")
    if (macAddress!= None):
        return macAddress.replace(":","")

    macAddress= get_mac_address(interface="wlan0")
    if (macAddress!= None):
        return macAddress.replace(":","")

    return "xxxxxxxx"


macAddress            = findMacAddress()
latestDisplayOn       = False
latestOn              = False

mintsDefinitions          = yaml.load(open('mintsXU4/mintsDefinitions.yaml'),Loader=yaml.FullLoader)
dataFolder                = mintsDefinitions['dataFolder']

dataFolderReference       = dataFolder +"/reference"
dataFolderMQTTReference   = dataFolder +"referenceMQTT"
dataFolderRaw             = dataFolder +"raw"
dataFolderMQTT            = dataFolder +"rawMQTT"
rawPklsFolder             = dataFolder    + "/rawPkls"
referencePklsFolder       = dataFolder    + "/referencePkls"
mergedPklsFolder          = dataFolder    + "/mergedPkls"
modelsPklsFolder          = dataFolder    + "/modelsPkls"
liveFolder                = dataFolder    + "/liveUpdate/results"


# For MQTT 
mqttOn                    = True
mqttCredentialsFile       = 'mintsXU4/credentials/credentials.yml'
mqttLoRaCredentialsFile   = 'mintsXU4/credentials/loRacredentials.yml'
portIDsFile               = 'mintsXU4/credentials/portIDs.yml'
mqttBroker                = "mqtt.circ.utdallas.edu"
mqttBrokerLoRa            = "mqtt.lora.trecis.cloud"
mqttPort                  = 8883  # Secure port
mqttPortLoRa              = 1883  # Secure port
# Take this from a yaml file
appKey                    = "CDCC4B13264CEDF92BEA7953C4034E21"
tlsCert                   = mintsDefinitions['tlsCert']
timeSpan                  = mintsDefinitions['timeSpan']

nodeIDsPre               = yaml.load(open('mintsXU4/credentials/nodeIDs.yaml'),Loader=yaml.FullLoader)
nodeIDs                  = nodeIDsPre['nodeIDs']

loRaE5MiniPorts          = findPorts("CP2102N USB to UART Bridge Controller","PID=10C4:EA60")
canareePorts             = findPorts("Canaree PM","PID=10C4:EA60")

keys                     = yaml.load(open('mintsXU4/credentials/keys.yml'),Loader=yaml.FullLoader)

print("E5 Mini Ports:")
for dev in loRaE5MiniPorts:
    print("\t{0}".format(dev))
    
print("Canaree Ports:")
for dev in canareePorts:
    print("\t{0}".format(dev))






    


 


     

