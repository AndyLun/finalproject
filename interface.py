import serial
import time
import math
import sys
import paho.mqtt.client as paho
mqttc = paho.Client()

host = "localhost"
topic = "mbed"
port = 1883


def on_connect(self, mosq, obj, rc):
    print("Connected rc: " + str(rc))


def on_message(mosq, obj, msg):
	print("[Received] Topic: " + msg.topic +
	      ", Message: " + str(msg.payload) + "\n")


def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed OK")


def on_unsubscribe(mosq, obj, mid, granted_qos):
    print("Unsubscribed OK")


mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe
print("Connecting to " + host + "/" + topic)
mqttc.connect(host, port=1883, keepalive=60)

# XBee setting
serdev = '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600, timeout=3)

s.write("+++".encode())
char = s.read(2)
print("Enter AT mode.")
print(char.decode())

s.write("ATMY 0x770\r\n".encode())
char = s.read(3)
print("Set MY 0x770.")
print(char.decode())

s.write("ATDL 0x670\r\n".encode())
char = s.read(3)
print("Set DL 0x670.")
print(char.decode())

s.write("ATID 0x17\r\n".encode())
char = s.read(3)
print("Set PAN ID 0x17.")
print(char.decode())

s.write("ATWR\r\n".encode())
char = s.read(3)
print("Write config.")
print(char.decode())

s.write("ATMY\r\n".encode())
char = s.read(4)
print("MY :")
print(char.decode())

s.write("ATDL\r\n".encode())
char = s.read(4)
print("DL : ")
print(char.decode())

s.write("ATCN\r\n".encode())
char = s.read(3)
print("Exit AT mode.")
print(char.decode())

try:
	while True:
		text_file = open("log.txt", "a")
		line = s.readline()
		print(line)
		
		try:
			linedec = line.decode()
			text_file.write("%s\n" % line.decode())
		except:
			s.flushInput()
except KeyboardInterrupt:
	text_file.close()
