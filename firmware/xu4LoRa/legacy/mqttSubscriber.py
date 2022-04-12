

# MQTT Client demo
# Continuously monitor two different MQTT topics for data,
# check if the received data matches two predefined 'commands'
 
import paho.mqtt.client as mqtt
import ast
import datetime
import yaml
import collections
import json
from mintsXU4 import mintsSensorReader as mSR
from mintsXU4 import mintsDefinitions as mD


mqttPort            = mD.mqttPort
mqttBroker          = mD.mqttBroker
mqttCredentialsFile = mD.mqttCredentialsFile


# For mqtt 
credentials = yaml.load(open(mqttCredentialsFile))
connected   = False  # Stores the connection status
broker      = mqttBroker  
port        = mqttPort  # Secure port
mqttUN      = credentials['mqtt']['username'] 
mqttPW      = credentials['mqtt']['password'] 

decoder = json.JSONDecoder(object_pairs_hook=collections.OrderedDict)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
 
    # Subscribing in on_connect() - if we lose the connection and
    # reconnect then subscriptions will be renewed.
    nodeID = "0242f3f105a8"
    client.subscribe(nodeID+"/HM3301")
    client.subscribe(nodeID+"/OPCN3")
    client.subscribe(nodeID+"/BME280")
    client.subscribe(nodeID+"/MGS001")
    client.subscribe("mintsTest")
    

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print()
    print(" - - - MINTS DATA RECEIVED - - - ")
    print()
    # print(msg.topic+":"+str(msg.payload))
    [nodeID,sensorID ] = msg.topic.split('/')
    sensorDictionary = decoder.decode(msg.payload.decode("utf-8","ignore"))
    dateTime  = datetime.datetime.strptime(sensorDictionary["dateTime"], '%Y-%m-%d %H:%M:%S.%f')
    writePath = mSR.getWritePathMQTT(nodeID,sensorID,dateTime)
    exists    = mSR.directoryCheck(writePath)
    sensorDictionary = decoder.decode(msg.payload.decode("utf-8","ignore"))
    print("Writing MQTT Data")
    print(writePath)
    mSR.writeCSV2(writePath,sensorDictionary,exists)
    print("Node ID   :" + nodeID)
    print("Sensor ID :" + sensorID)
    print("Data      : " + str(sensorDictionary))


# Create an MQTT client and attach our routines to it.
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(mqttUN,mqttPW)
client.connect(broker, port, 60)
client.loop_forever()
