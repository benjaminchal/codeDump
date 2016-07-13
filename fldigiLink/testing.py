import xmlrpc.client
import time, sys, threading, datetime, shutil
import sqlite3

#import crc16 <-- This is for the final linux build, cba to install this on PC

# should be localhost below , using example.com avoids email spam filters.
fldigi = "http://localhost:7362/RPC2"

server = xmlrpc.client.ServerProxy(fldigi)

def checkString(string): #THIS NEEDS IMPROVEMENT, PLAN TO USE THE CHECKSUM
    if len(string) > 10:
        if string[-6] == "*":
            return 1
        else:
            return 0
    else:
        return 0

def getRecentRTTY():
    data = server.text.get_rx(0,server.text.get_rx_length())
    data = str(data)
    data = data.split('$$')
    length = len(data)
    found = 0
    checkInt = 2
    string = str(data[-checkInt])
    while checkString(string) == 0:
        checkInt = checkInt + 1
        string = str(data[-checkInt])
    string = string.split('*')
    string = str(string[0])
    return(string)

def splitTelemData(string):
    #telemData = string.split('*') <-- Will be made irrelevant as I will remove checksums in checkString()
    telemData = string.split(',')
    telemData = {'time': telemData[2], 'latitude': telemData[3], 'longditude': telemData[4], 'altitude': telemData[5]}
    return telemData

def printAltitude():
    telemData = splitTelemData(getRecentRTTY())
    print('Altitude:', telemData['altitude'])


def printPosition():
    telemData = splitTelemData(getRecentRTTY())
    print('Latitude:', telemData['latitude'])
    print('Longditude:', telemData['longditude'])

def printTime():
    telemData = splitTelemData(getRecentRTTY())
    print('Time:', telemData['time'])

def userInput():
    # runs in main loop looking for user commands
    tester = input()
    if tester == 'a':
        printAltitude()
    if tester == 'p':
        printPosition()
    if tester == 'e':
        sys.exit()
    if tester == 't':
        printTime()
print(fldigi)
version = server.fldigi.version()
print('CONNECTING TO fldigi',version)
while 1:
    userInput()