# -*- coding: utf-8 -*-

# echo-client.py

import socket
import binascii
import time

HOST = "192.168.1.7"  # The server's hostname or IP address
PORT = 20001  # The port used by the server


def showValues(x):
    print("Enc 1: " + str(x[0]) + "   " + "Enc 2: " + str(x[1]) + "   " + "Enc 3: " + str(x[2]) + "   " + "Enc 4: " + str(x[3]))
    
def angleCalculation(byte1, byte2):
    conc_number = str(byte1) + str(byte2)
    dec = int(conc_number, 16)
    angle = (360.0/4095) * dec
    angle = round(angle, 1)
    return angle 
    
def decode(data):
     data_decoded = bytes.hex(data, '-')
     #print(data_decoded)
     # When splitting the bytes, the result is a string
     if int(data_decoded.split("-")[4]) == 67:
         byte1 = data_decoded.split("-")[5]
         byte2 = data_decoded.split("-")[6]
         enc1 = angleCalculation(byte1, byte2)
         
         byte1 = data_decoded.split("-")[7]
         byte2 = data_decoded.split("-")[8]
         enc2 = angleCalculation(byte1, byte2)
         
         byte1 = data_decoded.split("-")[9]
         byte2 = data_decoded.split("-")[10]
         enc3 = angleCalculation(byte1, byte2)
         
         byte1 = data_decoded.split("-")[11]
         byte2 = data_decoded.split("-")[12]
         enc4 = angleCalculation(byte1, byte2)
         return [enc1, enc2, enc3, enc4]
     


setup_array = [0x01, 0x00, 0x00, 0x00, 0x42, 0x1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
message = bytearray()
for i in setup_array:
        message.append(i)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print("Connected")
    while True:
        s.sendall(message) # Send data
        data = s.recv(13)
        angles = decode(data)
        showValues(angles)
        time.sleep(0.3)    # sleep 1 second


