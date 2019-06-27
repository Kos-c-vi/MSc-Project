import serial
import time

serial_port = serial.Serial('COM7', 9600, timeout=1)

counter = 0
#oldreadCommand    = '\xAF\xAF\x00\x00\xAF\x80\x02\x02\x00\x00\x91\x0D\x0A'#this checks other lora's behaviour?
readCommand    = '\xAF\xAF\x00\x00\xAF\x80\x02\x0D\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x9C\x0D\x0A'#this checks other lora's behaviour?
#				      \\0e\\00\\00\\00\\00\\00\\00\\00\\00\\00\\00\\00\\00\\00\\00\\9d\\0d\\0a
#response:        \\af\\af\\00\\00\\af\\00\\02\\0c\\04\\00\\d9\\60\\24\\0b\\00\\07\\00\\00\\00\\07\\95\\0d\\0a
writeCommand   = '\xaf\xaf\x00\x00\xaf\x80\x01\x0c\x04\x00\xd9\x60\x24\x0b\x00\x07\x00\x00\x00\x07\x14\x0d\x0a'#this tells all the other lora's how to behave?
#\x4a\x61\x73\x6f\x6e\x3a\x20\x61\x62\x63
#writeCommand = writeCommand.encode("hex")
#print(writeCommand)
#writeCommand  = '\xAF\xAF\x00\x00\xAF\x80\x01\x0C, ...CS\X0D\x0'

read_RSSI = '\xAF\xAF\x00\x00\xAF\x80\x06\x02\x00\x00\x95\x0D\x0A'

baudrate = 0
message = ''
data = ''

##setup the LoRa device by changing the variables below, then sending them


syncword	= '\xaf\xaf'     #always
IDcode		= '\x00\x00'     #default
header		= '\xaf'         #defailt
command		= '\x80\x01'     #x80 because we are sending the command (not receiving), x01 because it is a write/instruct command (not read)	
length		= '\x0c'         #for LoRa software version 2.2, always length = 12 bytes
baudrate	= '\x04'         #04 = 9600 baud
parity		= '\x00'         #default - no partiy
frequency	= '\xd9\x60\x24' #(this number in decimal) * 61.035 gives the frequency in Hz 
RFfactor	= '\x0c'         #default is x0b = 12 = 2048
mode		= '\x00'         #x00=standard, x01=cenral, x02=node
RFbandwidth	= '\x07'         #
NodeID		= '\x00\x00'     #default(dont change it?) x00x00 - xffxff
NetID		= '\x00'         #default x00 - xff
RFpower		= '\x07'         #x01=4dBm, x02=7dBm, x03=10dBm, x04=13dBm, x05=14dBm, x06=17dBm, x07=20dBm
CRC             = ''#CRC to be calculated
endcode		= '\x0d\x0a'     #standard




def calculateCRC():
	CRC = syncword + IDcode + header + command + length + baudrate + parity + frequency + RFfactor + mode + RFbandwidth + NodeID + NetID + RFpower
	#t = iter(CRC)
	#CRC = 'x'.join(a+b for a,b in zip(t, t))
	#CRC = CRC.split("x")
	#CRC = hex(int(baudrate,16))+hex(int(parity,16))+hex(int(frequency,16))+hex(int(RFfactor,16))+hex(int(mode,16))+hex(int(RFbandwidth,16))+hex(int(NodeID,16))+hex(int(NetID,16))+hex(int(RFpower,16))	
	#CRC = int(baudrate,16) + int(parity,16) + int(frequency,16) + int(RFfactor,16) + int(mode,16) + int(RFbandwidth,16) + int(NodeID,16) + int(NetID,16) + int(RFpower,16)
	#print hex(int(length,16)) #hex number
	#print length	#words
	#CRC = length + baudrate
	
	#CRC = CRC % 256
	#CRC = hex(CRC)
	t = iter(CRC.encode("hex"))
	CRC = 'x'.join(a+b for a,b in zip(t, t))
	CRC = CRC.split("x")
	print CRC
	i = 0
	total = 0
	while i < 20: #standard length of write command in v2.2
		total = total + int(CRC[i], 16)
		i = i + 1
	total = total % 256
	print total	
	total = hex(total)
	CRC = ''
	CRC = total
	print CRC
	#print CRC.encode("hex")
	#print header
	#print header.encode("hex")
	#print "CRC: ", CRC.encode("hex")

def makeWriteCommand():
	writeCommand = syncword + IDcode + header + command + length + baudrate + parity + frequency + RFfactor + mode + RFbandwidth + NodeID + NetID + RFpower + CRC + endcode
	print writeCommand
	print "writeCommand: ", writeCommand.encode("hex")


calculateCRC()
makeWriteCommand()

array = 0
serial_port.flushInput() 
while counter == 0:
	
	

	serial_port.write(writeCommand)
	#serial_port.write(read_RSSI)
	
	while serial_port.inWaiting() == 0:
		#print "waiting..."
		pass

	#serial_port.write(readCommand)
	
	result = serial_port.read(2500)
	print(result)
	#result = readCommand
	result = result.encode("hex")
	print "result: ", result
	print "result length: ", len(result)

	t = iter(result)
	result = 'x'.join(a+b for a,b in zip(t, t))
	result = result.split("x")
	
		
	print(result)
	
	resultLength = len(result)

	#find Sync Word
	i = 0	
	while i <= resultLength:
		
		#print(result[i])
		#print()
		if ((result[i] == 'af') and (result[i+1] == 'af')):
			sw = i
			print "syncword position: ", sw
			break
		i += 1	
	j = 0	
	while j < sw:
		message = message + result[j].decode('hex')
		j += 1	
	#print"message: ",message
 
	result[sw]	#sync word
	result[sw+1]	#sync word
	result[sw+2]	#ID code
	result[sw+3]    #ID code
	print "ID Code      : ", result[sw+2],result[sw+3]
	result[sw+4]	#header
	result[sw+5]	#command XX
	print "Command XX   : ", result[sw+5], "(received)"
	result[sw+6]	#command YY
	print "Command YY   : ", result[sw+6],
	if (result[sw+6] == '01'):
		print "(type: write)"
	if (result[sw+6] == '02'):
		print "(type: read)"
	result[sw+7]	#length~
	length = int(result[sw+7],16)
	print "length 	     : ", length," (",result[sw+7],")"
	
	k = 0
	while k < length:
		data = data + result[sw+7+k]
		k += 1
	
	


	result[sw+8]	#baudrate
	if (result[sw+8] == '04'):
		baudrate = 9600
	print "baudrate     : ", baudrate 
	result[sw+9]	#parity
	result[sw+10]	#frequency
	result[sw+11]	#frequency
	result[sw+12]	#frequency
	frequency = (int((result[sw+10]+result[sw+11]+result[sw+12]), 16)*61.035)/1000000
	print "frequency    : ",frequency, "MHz"
	result[sw+13]	#fr factor
	result[sw+14]	#mode
	result[sw+15]	#RF BW
	result[sw+16]	#ID (Node ID reserved?)
	result[sw+17]	#ID (Node ID reserved?)
	print "ID           : ", result[sw+16]+result[sw+17]
	result[sw+18]	#NetID
	print "Net ID       : ", result[sw+18]
	result[sw+19]	#RF power
	if (result[sw+19] == '01'):
		print("RF Power     :  4dBm")
	if (result[sw+19] == '02'):
		print("RF Power     :  7dBm")
	if (result[sw+19] == '03'):
		print("RF Power     :  10dBm")
	if (result[sw+19] == '04'):
		print("RF Power     :  13dBm")
	if (result[sw+19] == '05'):
		print("RF Power     :  14dBm")
	if (result[sw+19] == '06'):
		print("RF Power     :  17dBm")
	if (result[sw+19] == '07'):
		print("RF Power     :  20dBm")

####DATA
	result[resultLength-3]	#CRC~
	result[resultLength-2]	#end code
	result[resultLength-1]	#end code
	

	

	counter += 1                           

