import time
import serial
import msvcrt
import struct

class mapNode:
    numberOfNode = 0
    nodeType = 0
    northConnection = 0
    distNorth = 0
    southConnection = 0
    distSouth = 0
    eastConnection = 0
    distEast = 0

class BigInfoPackage:
    controlMode = 'FE'.decode('hex')
    escValue = '0000'.decode('hex')
    steeringValue = '0000'.decode('hex')
    distRightLine = 'FE'.decode('hex')
    angularDiff = 'FE'.decode('hex')
    ultraSonicFront = 'FE'.decode('hex')
    ultraSonicLeft = 'FE'.decode('hex')
    ultraSonicRight = 'FE'.decode('hex')
    ultraSonicBack = 'FE'.decode('hex')
    angle = 'FE'.decode('hex')
    speed = 'FE'.decode('hex')
    distToStopLine = 'FE'.decode('hex')
    signType = 'FE'.decode('hex')
    crossingType = 'FE'.decode('hex')
    nextTurnDecision = 'FE'.decode('hex')
    lastNode = 'FE'.decode('hex')
    nextNode = 'FE'.decode('hex')

def sendMap():

    #User inputs map
    print('-------------MAP----------')
    print("Input map info.")
    amountOfNodesStr = raw_input('Enter amount of nodes: ')
    amountOfNodes = int(amountOfNodesStr)
    map = [mapNode() for k in range(amountOfNodes)]

    for i in range(0, amountOfNodes):
        print("---Node " + str(i+1) + " ---")
        map[i].nodeType = int(raw_input('Enter node type: '))
        map[i].northConnection = int(raw_input('Node connected to north (0 if none):  '))
        if (map[i].northConnection == 0):
            map[i].distNorth = 10000
        else:
            map[i].distSouth = int(raw_input('Distance to nortern node:  '))

        map[i].southConnection = int(raw_input('Node connected to south (0 if none):  '))
        if (map[i].southConnection == 0):
            map[i].distSouth = 10000
        else:
            map[i].distSouth = int(raw_input('Distance to southern node:  '))

        map[i].eastConnection = int(raw_input('Node connected to east (0 if none):  '))
        if (map[i].eastConnection == 0):
            map[i].distEast = 10000
        else:
            map[i].distEast = int(raw_input('Distance to eastern node:  '))

    print('---------END OF MAP--------')
    print(' ')
    #Send map
    ser.write('FF'.decode('hex'))
    ser.write('01'.decode('hex'))

    for i in range(0, amountOfNodes):
        sendVar = struct.pack('<H', map[i].nodeType)
        ser.write(sendVar)
        sendVar = struct.pack('<H', map[i].northConnection)
        ser.write(sendVar)
        sendVar = struct.pack('<H', map[i].distNorth)
        ser.write(sendVar)
        sendVar = struct.pack('<H', map[i].southConnection)
        ser.write(sendVar)
        sendVar = struct.pack('<H', map[i].distSouth)
        ser.write(sendVar)
        sendVar = struct.pack('<H', map[i].eastConnection)
        ser.write(sendVar)
        sendVar = struct.pack('<H', map[i].distEast)
        ser.write(sendVar)

    ser.write('FE'.decode('hex'))

    return True;


def readSensInfo(ser, sensInfo):
    #Returns True if it has been read and False if failed reading
    #If there is something before header byte, it will get flushed

    if (ser.inWaiting() < 17):
        return False

    #Flush everyhing before header byte. This also reads the header byte
    #while(ser.read(size=1) != 'FF'.decode('hex')):
    #    if ser.inWaiting() < 16:
    #        return False
    if(ser.read(size=1) != 'FF'.decode('hex')):
        return False




    #read everything
    sensInfo.controlMode = ser.read(size=1)
    sensInfo.escValue = ser.read(size=2)
    sensInfo.steeringValue = ser.read(size=2)
    sensInfo.distRightLine = ser.read(size=1)
    sensInfo.angularDiff = ser.read(size=1)
    sensInfo.ultraSonicFront = ser.read(size=1)
    sensInfo.ultraSonicLeft = ser.read(size=1)
    sensInfo.ultraSonicRight = ser.read(size=1)
    sensInfo.ultraSonicBack = ser.read(size=1)
    #From here, checking for headerbyte in case information gone unmissed
    tempRead = ser.read(size=1)
    if (tempRead == 'FF'.decode('hex')):
        return False
    sensInfo.angle = tempRead

    tempRead = ser.read(size=1)
    if (tempRead == 'FF'.decode('hex')):
        return False
    sensInfo.speed = tempRead

    tempRead = ser.read(size=1)
    if (tempRead == 'FF'.decode('hex')):
        return False
    sensInfo.distToStopLine = tempRead

    tempRead = ser.read(size=1)
    if (tempRead == 'FF'.decode('hex')):
        return False
    sensInfo.nextTurnDecision = tempRead

    tempRead = ser.read(size=1)
    if (tempRead == 'FF'.decode('hex')):
        return False
    sensInfo.lastNode = tempRead

    tempRead = ser.read(size=1)
    if (tempRead == 'FF'.decode('hex')):
        return False
    sensInfo.nextNode = tempRead

    return True;

def userInputForMap():

    print("Input map info: ")
    print("Press Enter after every input. ")
    #return mapArray;

def sendInfoToComMod():
    #returns void

    #Remember to send everything in shorts. If something is sent with an character, send it with 00 before.
    pass

    return;

def printInfo(sensInfo):
    #Prints all the info on terminal

    #b"abcde".decode("utf-8") might work
    print(" ")
    print("|------------------------------------------")
    print("|Styrmodulens styrmod: " + str(struct.unpack("<B", sensInfo.controlMode)))
    #print("|Value input to ESC: " + str(struct.unpack(">H", sensInfo.escValue))) #Big Endian
    #print("|Value input to steering: " + str(struct.unpack(">H", sensInfo.steeringValue)))
    print("|Value input to ESC: " + str(struct.unpack("<H", sensInfo.escValue))) #Big Endian
    print("|Value input to steering: " + str(struct.unpack("<H", sensInfo.steeringValue)))
    print("|C-value: " + str(struct.unpack("<B", sensInfo.distRightLine)))
    print("|V-value: " + str(struct.unpack("<B", sensInfo.angularDiff)))
    print("|Distance from ultrasonics: ")
    print("|   Front: " + str(struct.unpack("<B", sensInfo.ultraSonicFront)))
    print("|   (Not in use) Left: " + str(struct.unpack("<B", sensInfo.ultraSonicLeft)))
    print("|   (Not in use) Right: " + str(struct.unpack("<B", sensInfo.ultraSonicRight)))
    print("|   Back: " + str(struct.unpack("<B", sensInfo.ultraSonicBack)))
    print("|Angle from gyrometer: " + str(struct.unpack("<B", sensInfo.angle)))
    print("|(Not in use) Speed: " + str(struct.unpack("<B", sensInfo.speed)))
    print("|Stop line detected (1 if detected, 0 otherwise): " + str(struct.unpack("<B", sensInfo.distToStopLine)))
    #print("|Signs that are seen: " + str(struct.unpack("<B", sensInfo.signType)))
    #print("|Crossing type ahead: " + str(struct.unpack("<B", sensInfo.crossingType)))
    print("|Which direction car will take in next crossing(102 or 70 for forward, 114 for right and 108 for left): " + str(struct.unpack("<B", sensInfo.nextTurnDecision)))
    print("|Last node visited: " + str(struct.unpack("<B", sensInfo.lastNode)))
    print("|Next node to visit: " + str(struct.unpack("<B", sensInfo.nextNode)))
    print("|------------------------------------------")

    return;



def sendKeyHits(ser, keyStrokes):
    #Send pressed keys and ends with a end byte ('e')
    #up, left, down and right are representeded by wasd
    #Sending a total of 3+(amount of keys pressed down) bytes

    bytesSent = 0

    #send hedaer byte
    theByte = 'FF'.decode('hex') #FF
    ser.write(theByte)

    #Send package-id byte
    theByte = '03'.decode('hex') #03
    ser.write(theByte)

    #Bit manipulation is done in Kommunikationsmodul
    if keyStrokes[0]:  # Wheel point to left 'a'
        ser.write('a')
        bytesSent += 1
    if keyStrokes[1]:  # Wheel point to right 'd'
        ser.write('d')
        bytesSent += 1
    if keyStrokes[2]:  # drive forward 'w'
        ser.write('w')
        bytesSent += 1
    if keyStrokes[3]:  # reverse 's'
        ser.write('s')
        bytesSent += 1
    if keyStrokes[4]:  # Raise speed 'r'
        ser.write('r')
        bytesSent += 1
    if keyStrokes[5]:  # lower speed 'f'
        ser.write('f')
        bytesSent += 1
    if keyStrokes[6]:  # turn off car'x'
        ser.write('x')
        bytesSent += 1
    if keyStrokes[7]:  # quit manual mode / switch to autonomous mode 'q'
        ser.write('q')
        bytesSent += 1

    #Send end-byte
    theByte = 'FE'.decode('hex') #FE
    ser.write(theByte)


    return;

#--------Main program---------

#---Init---
#Including user inputting map
#and sending map to kom.modul
#Also including setting up bluetooth connection


ser = serial.Serial(
    port = 'COM3',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS,
    timeout=0,
    writeTimeout=None
)

print("Connected via: " + ser.name)

ser.flushInput()

ser.write('g')
readyToGo = 0
while(readyToGo == 0):
    print("Wait for response")
    if(ser.inWaiting() > 0):
        readyCheck = ser.read(size=1)
        print("Value has come in. Checking if correct readyValue")
        if(readyCheck == 'c'):
            readyToGo = 1
            print("Ready")

print("Connection established and init phase starting. Sending over destination list and then map")

#Sending over destination list. TESTING right now
ser.write('FF'.decode('hex'))
ser.write('04'.decode('hex'))
ser.write('01'.decode('hex'))
ser.write('03'.decode('hex'))
ser.write('06'.decode('hex'))
ser.write('FE'.decode('hex'))

#Checking if komm.modul received dest list
readyToGo = 0
while(readyToGo == 0):
    if(ser.inWaiting() > 0):
        readyCheck = ser.read(size=1)
        print("Value has come in. Checking if correct readyValue")
        if(readyCheck == 'm'):
            readyToGo = 1
            print("Dest list received")

# Send map - Hardcoded
ser.write('FF'.decode('hex'))
ser.write('01'.decode('hex'))

#--NODE 1--
ser.write('0400'.decode('hex')) #node type
ser.write('0200'.decode('hex')) #northern node
ser.write('0700'.decode('hex')) #dist N
ser.write('0000'.decode('hex')) #Southern n
ser.write('0010'.decode('hex')) #dist S
ser.write('0000'.decode('hex')) #east node
ser.write('0010'.decode('hex')) # dist E

#--NODE 2--
ser.write('0100'.decode('hex')) #node type
ser.write('0400'.decode('hex')) #northern node
ser.write('0200'.decode('hex')) #dist N
ser.write('0100'.decode('hex')) #Southern n
ser.write('0700'.decode('hex')) #dist S
ser.write('0300'.decode('hex')) #east node
ser.write('0500'.decode('hex')) # dist E

#--NODE 3--
ser.write('0400'.decode('hex')) #node type
ser.write('0400'.decode('hex')) #northern node
ser.write('0600'.decode('hex')) #dist N
ser.write('0200'.decode('hex')) #Southern n
ser.write('0500'.decode('hex')) #dist S
ser.write('0000'.decode('hex')) #east node
ser.write('0010'.decode('hex')) # dist E

#--NODE 4--
ser.write('0100'.decode('hex')) #node type
ser.write('0300'.decode('hex')) #northern node
ser.write('0600'.decode('hex')) #dist N
ser.write('0500'.decode('hex')) #Southern n
ser.write('0500'.decode('hex')) #dist S
ser.write('0200'.decode('hex')) #east node
ser.write('0200'.decode('hex')) # dist E

#--NODE 5--
ser.write('0400'.decode('hex')) #node type
ser.write('0400'.decode('hex')) #northern node
ser.write('0500'.decode('hex')) #dist N
ser.write('0600'.decode('hex')) #Southern n
ser.write('0200'.decode('hex')) #dist S
ser.write('0000'.decode('hex')) #east node
ser.write('0010'.decode('hex')) # dist E

#--NODE 6--
ser.write('0400'.decode('hex')) #node type
ser.write('0500'.decode('hex')) #northern node
ser.write('0200'.decode('hex')) #dist N
ser.write('0000'.decode('hex')) #Southern n
ser.write('0010'.decode('hex')) #dist S
ser.write('0000'.decode('hex')) #east node
ser.write('0010'.decode('hex')) # dist E

ser.write('FE00'.decode('hex')) # dist E
#sendMap()

#Checking if map recieved
readyToGo = 0
while(readyToGo == 0):
    if(ser.inWaiting() > 0):
        readyCheck = ser.read(size=1)
        print("Value has come in. Checking if correct readyValue")
        if(readyCheck == 'd'):
            readyToGo = 1
            print("Map recieved")

#True if hit, False if not. Index 0..7 represents a, d, w, s, r, f, x, q
keyStrokes = [False,False,False,False,False,False,False,False]

#It is True just for testing. Will init as False in final program
manualMode = False

#Time between send-function calls in manual mode
manSendT = 0.2

#Used for manualMode at the moment.
startTime = time.time()

#Init for all readSensInfo variables
#FYI, Every variable init value might be wrong
sensInfo = BigInfoPackage()


#Test setttings hehehehe
readInfo = False
fpsCounter = 0


#---Main Loop---
while True:

    #---Manual mode.---
    if(manualMode):

        # Reading keystrokes
        tempBufferAmount = ser.inWaiting()
        readInfo = readSensInfo(ser, sensInfo)
        tempWait = ser.inWaiting()
        if tempWait != 0:
            pass
        if readInfo:
            fpsCounter += 1
            bufferAmountWhenRead = tempBufferAmount
        if(readInfo and (time.time() - startTime > 0.3)):
            printInfo(sensInfo)
            fps = fpsCounter / (time.time() - startTime)
            startTime = time.time()
            print("Bytes in input buffer: " + str(ser.inWaiting()))
            print("Bytes in buffer when read: " + str(bufferAmountWhenRead))
            print("Reading freq. is: " + str(fps))
            fpsCounter = 0

        #if keys are hit
        while (msvcrt.kbhit()):
            readKeyHit = msvcrt.getch()

            #reading key values
            if(readKeyHit == 'a'):
                keyStrokes[0] = ~keyStrokes[0]
            if (readKeyHit == 'd'):
                keyStrokes[1] = ~keyStrokes[1]
            if (readKeyHit == 'w'):
                keyStrokes[2] = ~keyStrokes[2]
            if (readKeyHit == 's'):
                keyStrokes[3] = ~keyStrokes[3]
            if (readKeyHit == 'r'):
                keyStrokes[4] = True #set to false when sent
            if (readKeyHit == 'f'):
                keyStrokes[5] = True #set to false when sent
            if (readKeyHit == 'x'):
                keyStrokes[6] = True #set to false when sent
            if (readKeyHit == 'q'):
                keyStrokes[7] = True #set to false when sent

        #Checking how long time has elapsed, and if long enough it will send keystrokes
        endTime = time.time()
        if(endTime - startTime > manSendT):
            #Send-function
            sendKeyHits(ser, keyStrokes)

            #if user wants to switch mode
            if(keyStrokes[7]):
                manualMode = False
                theByte = 'FF'.decode('hex')  # header byte
                ser.write(theByte)
                theByte = '02'.decode('hex')  # package type 2
                ser.write(theByte)
                theByte = '00'.decode('hex')  # manual mode false
                ser.write(theByte)
                print("Changing mode to auto")

            #resetting some keys
            keyStrokes[4] = False
            keyStrokes[5] = False
            keyStrokes[6] = False
            keyStrokes[7] = False


            #setting new starttime
            startTime = time.time()



    #---------Autonmous mode-----------
    else:
        #Checking if mode has to change
        if (msvcrt.kbhit()):
            readKeyHit = msvcrt.getch()
            if(readKeyHit == 'q'):
                manualMode = True
                theByte = 'FF'.decode('hex')  # header byte
                ser.write(theByte)
                theByte = '02'.decode('hex')  # package type 2
                ser.write(theByte)
                theByte = '01'.decode('hex')  # manual mode false
                ser.write(theByte)
                print("Changing mode to manual mode")

        #In order to calc reads per second
        tempBufferAmount = ser.inWaiting()
        #Reading info
        readInfo = readSensInfo(ser, sensInfo)
        tempWait = ser.inWaiting()
        if tempWait != 0:
            pass
        if readInfo:
            fpsCounter += 1
            bufferAmountWhenRead = tempBufferAmount
        if(readInfo and (time.time() - startTime > 0.3)):
            printInfo(sensInfo)
            fps = fpsCounter / (time.time() - startTime)
            startTime = time.time()
            print("Bytes in input buffer: " + str(ser.inWaiting()))
            print("Bytes in buffer when read: " + str(bufferAmountWhenRead))
            print("Reading freq. is: " + str(fps))
            fpsCounter = 0




