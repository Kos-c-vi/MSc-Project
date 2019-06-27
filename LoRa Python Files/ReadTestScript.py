""""
Experiment Test: TX Logger Code
Node 0
By Kossivi Fangbemi
"""
import time
from datetime import datetime
import serial

Serial_Com = serial.Serial("COM7",9600, timeout = 1)

# Commands to read/write (select) the radio properties of the receiver
readCommand    = '\xAF\xAF\x00\x00\xAF\x80\x02\x0D\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x9C\x0D\x0A'#this checks other lora's behaviour?
readCommand    = '\xAF\xAF\x00\x00\xAF\x80\x02\x0D\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x9C\x0D\x0A' 
writeCommand   = '\xaf\xaf\x00\x00\xaf\x80\x01\x0c\x04\x00\xd9\x60\x24\x0b\x00\x07\x00\x00\x00\x07\x14\x0d\x0a'#this tells all the other lora's how to behave?
# Command to read tthe RSSI value of the received packet
Read_RSSI = '\xAF\xAF\x00\x00\xAF\x80\x06\x02\x00\x00\x95\x0D\x0A'


#try:
#    Serial_Com.open()

#except Exception, e:
#    print 'Serial Port not opned' + str(e)
Serial_Com.close()
time.sleep(1)
Serial_Com.open()
time.sleep(1)
Serial_Com.flushInput()

if Serial_Com.isOpen():
    print 'Com port open'
    Serial_Com.write(Read_RSSI)
    print 'Read Commend send'
    #time.sleep(0.01)
    while Serial_Com.inWaiting() == 0:
        #print "waiting"
        pass

    Data = Serial_Com.read(2500)
    if Data != '':
        #print 'Data read'
        print Data.encode("hex")

    else:
        print 'No feedback'

else:
    print 'Serial port not opened'
