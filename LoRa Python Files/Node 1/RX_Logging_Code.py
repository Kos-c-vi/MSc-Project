""""
Experiment Test: RX Logger Code
Node 1
By Kossivi Fangbemi
"""
import time
import serial

serial_com4 = serial.Serial("COM7", 9600, timeout= 0.30)
try:
    Text_File = open('RX_Logger_File', 'a')
    """Text_File.write('Nth Data sent \t || \t TX Power  dBm \t || \t Acknowledgment \t || \t RSSI dBm \t\t || \t SRN || \n\r')
"""
    print('RX_Logger_File opened')
except Exception, e1:
    print 'Error in openning TX_Logger_File' + str(e1)
    exit()

try:
    serial_com4.open
    print('Serial Port opened')
except Exception, e:
    print 'Error in openning Serial Port' + str(e)
    exit()
    
if serial_com4.isOpen():
    print('Serial Port Really opened')
    try:
        serial_com4.flushInput()
        serial_com4.flushOutput()
        time.sleep(0.001)

        while True:
            Text_File = open('RX_Logger_File', 'a')
            Response = serial_com4.read(150)
            print('Serial Port readiing')
            if Response != '':
                print(Response)
                Text_File.write(Response)
                time.sleep(0.002)
                Text_File.close()
                #serial_com5.flushInput()
                #serial_com5.flushOutput()
                #break
                print('Serial Port read and export to text file')
                print('Message rec:' + str(Response))
                serial_com4.write("Ack")
                print("Ack")

        serial_com4.close()
        Text_File.close()

    except Exception, ee:
        print("Serial Communication Error: " + str(ee))
else:

    print("<<<-- Serial Port Com Not Opened -->>>")
