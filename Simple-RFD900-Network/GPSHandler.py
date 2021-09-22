import socket
import struct
import time
from threading import Thread
import threading
from _thread import *
import GlobalVals

import subprocess
from subprocess import Popen, PIPE

import sys, os
import copy


sys.path.insert(1,'../utils')
from utils import get_port
from common import *
from common_class import *


#=====================================================
# Thread for local GPS logger socket connection 
#=====================================================
def GPSLoggerSocket():

    # set up socket
    while True:
        try:
            socket_logger = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
            socket_logger.connect((GlobalVals.HOST, GlobalVals.GPS_LOGGER_SOCKET))
            socket_logger.settimeout(GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT)
            
        except Exception as e:
            if e.args[1] == 'Connection refused':
                print('Retry connecting to GPS....')
                time.sleep(1)
                continue
            else:
                print("Exception: " + str(e.__class__))
                print("There was an error starting the EKFlogger socket. This thread will now stop.")
                with GlobalVals.BREAK_GPS_LOGGER_THREAD_MUTEX:
                    GlobalVals.BREAK_GPS_LOGGER_THREAD = True
                return 
        break
    # intialize variables 
    bufferRead = 1024
    breakMainThread = False
    while True:
        
        # if flag is set break the thread 
        with GlobalVals.BREAK_GPS_LOGGER_THREAD_MUTEX:
            if GlobalVals.BREAK_GPS_LOGGER_THREAD:
                break

        # read the socket 
        while True:
            try:
                data_bytes = socket_logger.recv(bufferRead)
                # print("EKF socket received")
                break
            except Exception as e:
                if e.args[0] == 'timed out':
                    print("GPSLoggerSocket() Receive timed out. Retrying...")
                    time.sleep(0.1)
                    continue
                else:
                    print("GPSLoggerSocket(): Receive Connection error.")
                    breakMainThread = True
                    break
                
        if breakMainThread:
            break

        # if there is nothing in the socket then it has timed out 
        if len(data_bytes) == 0:
            continue

        data_str = data_bytes.decode('utf-8')
        string_list = extract_str_btw_curly_brackets(data_str)
        
        if len(string_list) > 0:
            gps_list = []
            manoeuvre_list = []

            for string in string_list:
                # print("!!!!!!!!!!!!!!!!!!!!!",string)
                received, gps_i, manoeuvre = stringToGPS_Manoeuvre(string)
                if received:
                    gps_list.append(gps_i)
                    manoeuvre_list.append(manoeuvre)

            idx = 0
            with GlobalVals.GPS_LOG_MUTEX:
                while idx < len(gps_list):
                    GlobalVals.GPS_ALL = [gps_list[idx]]
                    GlobalVals.MANOEUVRE = manoeuvre_list[idx]
                    idx += 1
            
        # pause a little bit so the mutexes are not getting called all the time 
        time.sleep(0.01)  

    socket_logger.close()
    return 

#=====================================================
# Thread for distributing GPS info to other scripts 
#=====================================================


def GPSDistributor():

    Distro_Socket = [None]*GlobalVals.N_NODE_PUBLISH
    Distro_Connection = [None]*GlobalVals.N_NODE_PUBLISH
    for i in range(GlobalVals.N_NODE_PUBLISH):
        # start socket 
        Distro_Socket[i] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
        Distro_Socket[i].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1) 
        Distro_Socket[i].bind((GlobalVals.HOST, GlobalVals.GPS_DISTRO_SOCKET[i]))
        Distro_Socket[i].settimeout(GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT)
        

        # Wait for connection on the distro socket 
        try:
            Distro_Socket[i].listen(1) 
            Distro_Connection[i], addr = Distro_Socket[i].accept()  
            Distro_Connection[i].settimeout(GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT) 
            print("Logger[",i,"] Connected to ", addr)                                            
        except Exception as e:
            print("Exception: " + str(e.__class__))
            print("Error in the GPSDistributor[",i,"] logger socket. Now closing thread.")
            with GlobalVals.BREAK_GPS_DISTRO_THREAD_MUTEX:
                GlobalVals.BREAK_GPS_DISTRO_THREAD = True
            return 0
  
    
    source1 = False
    source2 = False
    breakThread = False

    while True:

        # check if the thread need to break 
        if breakThread:
            break

        with GlobalVals.BREAK_GPS_DISTRO_THREAD_MUTEX:
            if GlobalVals.BREAK_GPS_DISTRO_THREAD:
                break

        # check if local GPS data has been recived 
        with GlobalVals.RECIEVED_GPS_LOCAL_DATA_MUTEX:
            if GlobalVals.RECIEVED_GPS_LOCAL_DATA:
                source1 = True
                GlobalVals.RECIEVED_GPS_LOCAL_DATA = False
        
        # check if GPS data from the radio has been recieved 
        with GlobalVals.RECIEVED_GPS_RADIO_DATA_MUTEX:
            if GlobalVals.RECIEVED_GPS_RADIO_DATA:
                source2 = True
                GlobalVals.RECIEVED_GPS_RADIO_DATA = False
        
        # if no data has been recieved sleep and loop
        if not source1 and not source2:
            time.sleep(0.1)
            continue
        else:
            source1 = False
            source2 = False

        with GlobalVals.GPS_DATA_BUFFER_MUTEX:
            while len(GlobalVals.GPS_DATA_BUFFER) > 0:

                # get the GPS data
                GPSData = GlobalVals.GPS_DATA_BUFFER.pop(0)
                Longitude = GPSData.Longitude
                Latitude = GPSData.Latitude
                Altitude = GPSData.Altitude
                GPSTime = int(GPSData.GPSTime)
                SystemID = GPSData.SystemID

                # create message string 
                messageStr = "{'system': " + str(SystemID) + "; 'altitude': " + str(Altitude) + "; 'latitude': " + str(Latitude) + "; 'longitude': " + str(Longitude) + "; 'time': " + str(GPSTime) + "}"
                messageStr_bytes = messageStr.encode('utf-8')

                # send the message 
                for i in range(GlobalVals.N_NODE_PUBLISH):
                    try:
                        Distro_Connection[i].sendall(messageStr_bytes)
                    except Exception as e:
                        print("Exception: " + str(e.__class__))
                        print("Error when sending to Distro_Connection[",i,"]. Now closing thread.")
                        breakThread = True
                        break
                
    for i in range(GlobalVals.N_NODE_PUBLISH):
        Distro_Connection[i].close()


def GPS_FormatCheck(GPSdata):
    errString = []
    err = False
    if not GPSdata.SystemID in GlobalVals.REAL_BALLOON:
        errString.append("GPSdata.SystemID: " + str(GPSdata.SystemID))
        err = True
    
    if GPSdata.SystemID == GlobalVals.SYSTEM_ID:
        errString.append("GPSdata.SystemID must be different")
        err = True

    if not valueInRange(GPSdata.Longitude,[-180,180]):
        errString.append("Longitude: " + str(GPSdata.Longitude))
        err = True

    if not valueInRange(GPSdata.Latitude,[-90,90]):
        errString.append("Latitude: " + str(GPSdata.Latitude))
        err = True
    
    if not valueInRange(GPSdata.Altitude,[-100,50000]):
        errString.append("Altitude: "+str(GPSdata.Altitude))
        err = True

    if not valueInRange(GPSdata.GPSTime,[GlobalVals.EXPERIMENT_TIME,None]):
        errString.append("Epoch: " + str(GPSdata.GPSTime))
        err = True
    if err:
        # print(errString)
        return False
    else:
        return True


#=====================================================
# Thread for distributing GPS info to other scripts 
#=====================================================


def GPSDistributorUDP():

    Distro_Socket = [None]*GlobalVals.N_REAL_BALLOON
    msg = [[] for _ in range(GlobalVals.N_REAL_BALLOON)]
    msgBytes = [[] for _ in range(GlobalVals.N_REAL_BALLOON)]
    for i in range(GlobalVals.N_REAL_BALLOON):
        emptyMsg = ''
        msgBytes[i] = emptyMsg.encode('utf-8')

    # start socket 
    for i in range(GlobalVals.N_REAL_BALLOON):
        Distro_Socket[i] = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        Distro_Socket[i].settimeout(GlobalVals.GPS_LOGGER_SOCKET_TIMEOUT)

    breakThread = False
    while True:

        with GlobalVals.BREAK_PLAYBACK_MUTEX:
            if GlobalVals.BREAK_PLAYBACK:
                breakThread = True
                
        with GlobalVals.BREAK_GPS_DISTRO_THREAD_MUTEX:
            if GlobalVals.BREAK_GPS_DISTRO_THREAD:
                break
        # check if the thread need to break 
        if breakThread:
            break

        with GlobalVals.GPS_LOG_MUTEX:
            GPS_Log = copy.deepcopy(GlobalVals.GPS_ALL)

        if not checkAllGPS(GPS_Log[0:GlobalVals.N_REAL_BALLOON]):
            time.sleep(0.1)
            continue

        # timeNow = round(time.time()*1000)
        # get the GPS data
        for i in range(GlobalVals.N_REAL_BALLOON):
            lat = GPS_Log[i].lat
            lon = GPS_Log[i].lon
            alt = max(GPS_Log[i].alt,350)

            timeNow = round(GPS_Log[i].epoch*1000)
            if lat == 0 or lon == 0:
                continue
            msg[i] = "{DATA_TYPE:MS_EPOCH;VALUE:" + str(timeNow) + "}{DATA_TYPE:GPS_LLA_ARRAY;VALUE:[" + str(lat) + "," + str(lon) + "," + str(alt) + "]}" 
            # print(msg[i])
            msgBytes[i]=msg[i].encode('utf-8')

        for i in range(GlobalVals.N_REAL_BALLOON):
            try:
                # Logger_Connection.sendall(socketPayload)
                Distro_Socket[i].sendto(msgBytes[i],('127.0.0.1',GlobalVals.GPS_UDP_DISTRO_SOCKET[i]))
                # print("GPS UDP SENDING TO ",i)
                # print(msg[i])
            except Exception as e:
                print("Exception: " + str(e.__class__))
                print("Error in the  UDPlogger socket. Now closing thread.")
                breakThread = True
                break

        time.sleep(0.001)

        
    for i in range(GlobalVals.N_REAL_BALLOON):
        print("Socket [",i,"] closed!")
        Distro_Socket[i].close()


def startPathPredictionProgram(i):
    
    # print("SYS",i)
    os.system('cd ~/firefly-programs/gs-path-prediction/build/ && ~/firefly-programs/gs-path-prediction/build/gs-path-prediction 127.0.0.1 '+ str(GlobalVals.GPS_UDP_DISTRO_SOCKET[i]) +" "+ str(i+1))
    # print(1)

def killPathPredictionProgram():
    for i in range(GlobalVals.N_REAL_BALLOON):
        os.system('fuser -k '+ str(GlobalVals.GPS_UDP_DISTRO_SOCKET[i]) +"/udp")
