# MQTT Client demo
# Continuously monitor two different MQTT topics for data,
# check if the received data matches two predefined 'commands'
import base64
from traceback import print_tb
import paho.mqtt.client as mqtt
import datetime
import yaml
import collections
import json

from mintsXU4 import mintsSensorReader as mSR
from mintsXU4 import mintsDefinitions as mD
from mintsXU4 import mintsLatest as mL
from mintsXU4 import mintsLoRaReader as mLR
from mintsXU4 import mintsLiveNodes as mLN
from collections import OrderedDict
import struct
import datetime

mqttPort            = mD.mqttPortLoRa
mqttBroker          = mD.mqttBrokerLoRa
mqttCredentialsFile = mD.mqttLoRaCredentialsFile
nodeIDs             = mD.nodeIDs


tlsCert             = mD.tlsCert

portIDsFile         = mD.portIDsFile
portDefinitions     = yaml.load(open(portIDsFile),Loader=yaml.FullLoader)
portIDs             = portDefinitions['portIDs']

credentials  = yaml.load(open(mqttCredentialsFile),Loader=yaml.FullLoader)
connected    = False  # Stores the connection status
broker       = mqttBroker  
port         = mqttPort  # Secure port
mqttUN       = credentials['mqtt']['username'] 
mqttPW       = credentials['mqtt']['password'] 
nodeObjects  = []
decoder = json.JSONDecoder(object_pairs_hook=collections.OrderedDict)

def getNodeIndex(nodeIDIn):
    indexOut = 0
    for node in nodeIDs:
        nodeID = node['nodeID']
        if (nodeID == nodeIDIn):
            return indexOut; 
        indexOut = indexOut +1
    return -1;

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    topic = "utd/lora/app/2/device/+/event/up"
    client.subscribe(topic)
    print("Subscrbing to Topic: "+ topic)
    for node in nodeIDs:
        print("Appending  Node")
        nodeID = node['nodeID']
        print(nodeID)
        nodeObjects.append(mLN.node(nodeID))
    
def on_message(client, userdata, msg):
    # try:
        print()
        print(" - - - MINTS DATA RECEIVED - - - ")
        # print(msg.payload)
        dateTime,gatewayID,nodeID,sensorID,framePort,base16Data = \
            mLR.loRaSummaryReceive(msg,portIDs)
        print("Node ID         : " + nodeID)
        nodeIndex = getNodeIndex(nodeID)
        if nodeIndex >= 0 :  
            print("============")
            sensorDictionary = mLR.sensorReceiveLoRa(dateTime,nodeID,sensorID,framePort,base16Data)
            dateTime = datetime.datetime.strptime(sensorDictionary["dateTime"], '%Y-%m-%d %H:%M:%S.%f')
            print("Node ID         : " + nodeID)
            print("Gateway ID      : " + gatewayID)
            print("Sensor ID       : " + sensorID)
            print("Date Time       : " + str(dateTime))
            print("Port ID         : " + str(framePort))
            print("Base 16 Data    : " + base16Data)

    # except Exception as e:
    #     print("[ERROR] Could not publish data, error: {}".format(e))


# Create an MQTT client and attach our routines to it.
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(mqttUN,mqttPW)
client.connect(broker, port, 60)
client.loop_forever()