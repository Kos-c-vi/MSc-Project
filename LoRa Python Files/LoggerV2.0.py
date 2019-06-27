""""
Experiment Test: TX Logger Code
Node 0
By Kossivi Fangbemi
"""
import time
from datetime import datetime
import serial

Serial_Com = serial.Serial("COM7", timeout = 1)

# Commands to read/write (select) the radio properties of the receiver
readCommand    = '\xAF\xAF\x00\x00\xAF\x80\x02\x0D\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x9C\x0D\x0A'#this checks other lora's behaviour?
writeCommand   = '\xaf\xaf\x00\x00\xaf\x80\x01\x0c\x04\x00\xd9\x60\x24\x0b\x00\x07\x00\x00\x00\x07\x14\x0d\x0a'#this tells all the other lora's how to behave?
# Command to read tthe RSSI value of the received packet
Read_RSSI = '\xAF\xAF\x00\x00\xAF\x80\x06\x02\x00\x00\x95\x0D\x0A'

# Defining Variables
Error_Rate = 0
Error_Rate_Old = 0
error = 0
error_Old = 0
counter = 0
counter_error = 0
Data_New = 0

# Openning the Log file
try:
    Text_File = open('Exp_Logger_File', 'a')
    Text_File.write('\n\r' + str(datetime.now()) + '\n\r')
    Text_File.write('\n\r Counter \t Nth Data Received \t RSSI dBm \t Error Rate \n\r')
    print('TX_Logger_File opened')
except Exception, e1:
    print 'Error in openning TX_Logger_File' + str(e1)
    exit()

# Openning the serial port 
try:
    Serial_Com.open
except Exception, e:
    print 'Error in openning Serial Port' + str(e)
    exit()

if Serial_Com.isOpen():
    print 'Serial port openned'
    try:
        Serial_Com.flushInput()
        Serial_Com.flushOutput()
        time.sleep(0.0001)

        while True:
            Data_old = Data_New
            Text_File = open('Exp_Logger_File', 'a')
            Data = Serial_Com.read(2500)
            #print 'Data read'
            print Data.encode("hex")
            
            if Data != '':
                #print Data
                time.sleep(0.0002)
                Data_New = int(Data.encode("hex"), 16)
                
                # Requesting the RSSI Value
                Serial_Com.write(Read_RSSI)

                # Reading the RSSI value
                RSSI_resp = Serial_Com.read(2500)

                if RSSI_resp != '':
                    i = iter(RSSI_resp.encode("hex"))
                    RSSI_Resp = ' '.join(a+b for a,b in zip(i, i))
                    RSSI_Resp = RSSI_Resp.split(' ')
                    #print RSSI_Resp
                    RX_Power_Factor = int(RSSI_Resp[8], 16)

                    # Calculation The RSSI
                    RX_Power = -164 + RX_Power_Factor 
                    print RX_Power
                    
                if int(Data.encode("hex"), 16) == 0:
                    counter = 0
                    Error_Rate = 0
                    error = 0

                # Checking the recieved data and calculation the error rate
                if int(Data.encode("hex"), 16) != counter:
                    error = int(Data.encode("hex"), 16) - counter
                    counter = int(Data.encode("hex"), 16)
                    if Data_New < Data_old:
                        # Resetting the error rate
                        Serial_Com.write(0)
                        error_Old = 99.00 - Data_old
                        Error_Rate_Old = Error_Rate + error_Old
                        error_Old = 0
                        Error_Rate = Data_New
                    else:
                    ##Sending ack back
                        Serial_Com.write('Ack')
                        Error_Rate = Error_Rate + error
                        error = 0
                    
                ## Logging Data
                Text_File.write(str(counter) + '\t' + '0x' + str(Data.encode("hex")) + '\t')
                Text_File.write(str(RX_Power) + '\t' )
                if Data_New < Data_old:
                    Text_File.write(str(Error_Rate_Old) + str(Data_old)+ '\t\t')
                Text_File.write(str(Error_Rate) + '\n\r' )    
                Text_File.close()
                print counter

                # Incrementing the counter
                counter = counter + 1
                    
                #break
            else:
                #time.sleep(1)
                counter_error = counter_error + 1
                #Text_File.write( '\t\t .........  \t\n\r' )

        #Serial_Com.close()
        Text_File.close()

    except Exception, ee:
        print("Serial Communication Error: " + str(ee))
else:

    print("<<<-- Serial Port Com Not Opened -->>>")
