#!/usr/bin/python
#
# Program to read data from nRF24L01 
# and upload to mongolab database.
#
import os
import sys
import time
import signal
import pymongo
import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import spidev
                
# -------- # -------- # -------- # -------- #
#    ID    #        Value        #  Denom   #
# -------- # -------- # -------- # -------- #
#    ID    #        Value        #  Denom   #
# -------- # -------- # -------- # -------- #

# Each frame received by nrf24l01 is n * 4 bytes
# consisting of an 8bit identifier, 16bit 
# data field, and an 8bit denominator, to 
# provide the decimal part. 

# mongolab connection
url = "mongodb://public:passw0rd@ds027761.mongolab.com:27761/autogrow"
# nrf24l01 tx/rx address
pipes = [[0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2]]
# data identifiers
TID = 200
HID = 201

# Catch exit ctrl-c
def signal_handler(signal, frame):
    print '\nBye!'
    sys.exit(0)

if __name__=="__main__":
    try:
        client = pymongo.MongoClient(url)
        db = client.autogrow
        sensors = db.sensors
    except:
        print "Couldn't connect to mongolab"
        sys.exit(-1)

    GPIO.setmode(GPIO.BCM)
    signal.signal(signal.SIGINT, signal_handler)

    radio2 = NRF24(GPIO, spidev.SpiDev())
    radio2.begin(0, 17)

    radio2.setRetries(15,15)

    radio2.setPayloadSize(8)
    radio2.setChannel(0x60)
    radio2.setDataRate(NRF24.BR_250KBPS)
    radio2.setPALevel(NRF24.PA_MIN)

    radio2.setAutoAck(False)
    radio2.enableDynamicPayloads()

    radio2.openWritingPipe(pipes[0])
    radio2.openReadingPipe(1, pipes[1])

    radio2.startListening()
    radio2.stopListening()

    radio2.printDetails()
    radio2.startListening()

    while True:
        pipe = [0]
        while not radio2.available(pipe):
            time.sleep(10000/1000000.0)

        recv_buffer = []
        radio2.read(recv_buffer)
        print recv_buffer

        if recv_buffer[0] == TID:
            temp = ((recv_buffer[1] << 8) + recv_buffer[2]) / recv_buffer[3]
            #calibration?
            temp = temp + 2
            sensors.update({"name": "temp"}, {"name":"temp", "value":round(temp,1), "time":time.time()})
            print "temp = %ddegC" % temp
        if recv_buffer[4] == HID:
            humid = ((recv_buffer[5] << 8) + recv_buffer[6]) / recv_buffer[7]
            humid = humid - 30
            sensors.update({"name": "humid"}, {"name":"humid", "value":round(humid,1), "time":time.time()})
            print "humidity = %dpercent" % humid
        else:
           print "packet not recognised"
