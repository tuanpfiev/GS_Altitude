# import libraries 
import time
from threading import Thread
from multiprocessing import Process
import calendar
from datetime import datetime

# import files 
import GlobalVals

import GPSHandler
import sys, os
import copy

sys.path.insert(1,'../utils')
from utils import get_port
from navpy import lla2ecef
import numpy as np
import json
from common import *
from common_class import *
import random
import boto3
from threading import Lock
import csv
import subprocess
import math


import glob

global countGPS_Total

class Path:

    def __init__(self, filePath = None, sysID = None):

        self.filePath = filePath if filePath is not None else ""
        self.sysID = sysID if sysID is not None else 1
        self.trajectory = []
        
        with open(self.filePath) as csv_file:
            
            self.reader = csv.reader(csv_file, delimiter=',')
            
            for row in self.reader:
                gps = GPS(self.sysID,float(row[1]),float(row[2]),float(row[3]), float(row[0]))
                self.trajectory.append(gps)

        self.epoch_range = [self.trajectory[0].epoch, self.trajectory[-1].epoch]

    def getGPS(self, query_epoch):

        if query_epoch <= self.epoch_range[0]:
            return self.trajectory[0]

        if query_epoch >= self.epoch_range[-1]:
            return self.trajectory[-1]

        for i in range(len(self.trajectory)):
            if self.trajectory[i].epoch > query_epoch:
                idx = i-1
                break

        ratio = (query_epoch - self.trajectory[idx].epoch) / (self.trajectory[idx+1].epoch - self.trajectory[idx].epoch)

        lat_interp = self.trajectory[idx].lat + (self.trajectory[idx+1].lat - self.trajectory[idx].lat) * ratio
        lon_interp = self.trajectory[idx].lon + (self.trajectory[idx+1].lon - self.trajectory[idx].lon) * ratio
        alt_interp = self.trajectory[idx].lat + (self.trajectory[idx+1].alt - self.trajectory[idx].alt) * ratio

        return GPS(self.sysID,lat_interp,lon_interp,alt_interp,query_epoch)
    
    def getDistance(self, gps):

        predictedGPS = self.getGPS(gps.epoch)

        positionENU_Relative = positionENU(gps,predictedGPS)
        distance = math.sqrt(positionENU_Relative[0]**2+positionENU_Relative[1]**2)
        
        return distance

targets = [-36.8941959968898, 143.283248020983]
targetLocation = [GPS(None, targets[0], targets[1], 0)]

nFire = 0
fireLocation = [-37.0977562987919, 142.522598419168]
predDuration = []
plannedManAlt = [1500, 5000, 12500, 25000]



def pathPredictionThread():
    
    while True:
        with GlobalVals.BREAK_PLAYBACK_MUTEX:
            if GlobalVals.BREAK_PLAYBACK:
                break

        with GlobalVals.BREAK_PATH_PREDICTION_THREAD_MUTEX:
            if GlobalVals.BREAK_PATH_PREDICTION_THREAD:
                break

        loadInitPathAll = []
        balloonPathAll = []
        pathPredictionMsg = []
        if not GlobalVals.LOADED_INIT_MESSAGE:
            for i in range(GlobalVals.N_REAL_BALLOON):
                initPredFile,_ = getLatestPrediction("./initialPrediction/","Balloon_",i+1)
 
                if initPredFile == None:
                    continue

                loadInitPathAll.append(Path(initPredFile,i+1))
            
            for i in range(len(loadInitPathAll)):
                predDuration.append(loadInitPathAll[i].trajectory[-1].epoch - loadInitPathAll[i].trajectory[0].epoch)
                for j in range(len(loadInitPathAll[i].trajectory)):
                    each_balloon = {
                                'idP': str(loadInitPathAll[i].trajectory[j].sysID),
                                'latP': str(loadInitPathAll[i].trajectory[j].lat),
                                'lonP': str(loadInitPathAll[i].trajectory[j].lon)
                            }
                    pathPredictionMsg.append(each_balloon)
                    GlobalVals.PRED_MAX_ALT[i] = max(GlobalVals.PRED_MAX_ALT[i], loadInitPathAll[i].trajectory[j].alt)
            with GlobalVals.PATH_PREDICTION_MSG_MUTEX:
                GlobalVals.PATH_PREDICTION_MSG = pathPredictionMsg

            GlobalVals.LOADED_INIT_MESSAGE = True

        else:
            # Check if GPS are ready?
            while not GlobalVals.PATH_PREDICTION_RUN_ONCE:
                with GlobalVals.GPS_LOG_MUTEX:
                    GPS_Log = copy.deepcopy(GlobalVals.GPS_ALL)

                if checkAllGPS(GPS_Log[0:GlobalVals.N_REAL_BALLOON]):
                    break
                else:
                    print("GPS of all balloons are not ready, WAITING FOR THE DATA")

                time.sleep(1)

            # Start prediction
            
            for i in range(GlobalVals.N_REAL_BALLOON):
                lastestFile,_ = getLatestPrediction("../../../firefly-programs/gs-path-prediction/logs/","cosine_filtered_mercator_bid_",i+1)
               
                if lastestFile == None:
                    print("Find no prediction files...")
                    time.sleep(1)
                    continue
                try:
                    balloonPathAll.append(Path(lastestFile,i+1))
                except:
                    pass
            # print("Found prediction files... !!!!")
            with GlobalVals.PATH_PREDICTION_MUTEX:
                GlobalVals.PATH_PREDICTION = balloonPathAll

            GlobalVals.PATH_PREDICTION_RUN_ONCE = True

def update_GPS_Log(gps_data):
    index = gps_data.SystemID
    try:    
        GlobalVals.GPS_ALL[index-1]= GPS(gps_data.SystemID, gps_data.Latitude, gps_data.Longitude, gps_data.Altitude,gps_data.GPSTime)
    except:
        print('here-update_GPS_Log')


def gps_lambda_handler(credentials):
    
    stream_name = 'RMITballoon_Data'
    k_client = boto3.client('kinesis', 
                            region_name='ap-southeast-2',
                            aws_access_key_id=credentials['AccessKeyId'],
                            aws_secret_access_key=credentials['SecretKey'],
                            aws_session_token=credentials['SessionToken'])
    
    count_t = 0
    while True:

        with GlobalVals.GPS_LOG_MUTEX:
            GPS_Log = copy.deepcopy(GlobalVals.GPS_ALL)

        if count_t == 0:
            with GlobalVals.PATH_PREDICTION_MSG_MUTEX:
                aws_message = GlobalVals.PATH_PREDICTION_MSG[:]
            if len(aws_message)==0:
                print("Length of aws_message is zero!!! Continue ...")
                continue

            initial_message = {
                'nBalloon': str(GlobalVals.N_REAL_BALLOON),
                'targets': str(targets),
                'nFire': str(nFire),
                'fireLocation': str(fireLocation),
                'flightDuration': str(predDuration),
                'manAlt': str(plannedManAlt)
            }
            aws_message.append(initial_message)
        else:

            # Create predicted paths from initial predictions (needs to update)
            with GlobalVals.PATH_PREDICTION_MUTEX:
                balloonPathAll = GlobalVals.PATH_PREDICTION

            if not checkAllGPS(GPS_Log) or not GlobalVals.PATH_PREDICTION_RUN_ONCE or len(balloonPathAll)==0 or len(balloonPathAll) != GlobalVals.N_REAL_BALLOON:
                print("GPS OK?: ",checkAllGPS(GPS_Log), ", PathPredRunOnce?: ",GlobalVals.PATH_PREDICTION_RUN_ONCE, ", Length of balloonPathAll: ", len(balloonPathAll), )
                time.sleep(1)
                continue

            predictedPaths = []
            for i in range(len(balloonPathAll)):
                each_balloon = {
                    'idP': str(balloonPathAll[i].trajectory[0].sysID),
                    'latP': str(GPS_Log[i].lat ),
                    'lonP': str(GPS_Log[i].lon)
                }
                predictedPaths.append(each_balloon)
                for j in range(len(balloonPathAll[i].trajectory)):
                    each_balloon = {
                        'idP': str(balloonPathAll[i].trajectory[j].sysID),
                        'latP': str(balloonPathAll[i].trajectory[j].lat),
                        'lonP': str(balloonPathAll[i].trajectory[j].lon)
                    }
                    predictedPaths.append(each_balloon)
            aws_message = predictedPaths[:]

            for i in range(len(GPS_Log)):

                positionENU_RelativeTarget = positionENU(GPS_Log[i],targetLocation[i])
                targetOffset = math.sqrt(positionENU_RelativeTarget[0]**2 + positionENU_RelativeTarget[1]**2)
                    
                if i<GlobalVals.N_REAL_BALLOON:
                    comms = 1
                else:
                    comms = 1
                comms = min(max(comms,1e-6),1-1e-6)

                each_balloon = {
                    'id': str(GPS_Log[i].sysID),
                    't': str(GPS_Log[i].epoch),
                    'lat': str(GPS_Log[i].lat ),
                    'lon': str(GPS_Log[i].lon),
                    'alt': str(GPS_Log[i].alt),
                    'tar': str(targetOffset),
                    'comms': str(comms),
                    'man': str(GlobalVals.MANOEUVRE)
                }
                aws_message.append(each_balloon)

        print(json.dumps(aws_message))

        response = k_client.put_record(
                StreamName=stream_name,
                Data=json.dumps(aws_message),
                
                PartitionKey='telemetryData'
        )
        # print("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")
        # print(aws_message)
        # print("Publishing to AWS Kinesis Data ...")
        # print("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")

        count_t = count_t + 1
        time.sleep(3)

def cognito_login(username, password):
    region = 'ap-southeast-2'
    clientID = '6s77pp2bq57348s96kujudbes5'
    userPoolID = 'ZHCg4nUow'
    identityPoolID = 'b8f994a8-9f48-4efb-ac58-e4101703ee87'
    login = 'cognito-idp.' + region + '.amazonaws.com/' + region + "_" + userPoolID
    cogIdp_client = boto3.client('cognito-idp', region_name=region)
    cog_client = boto3.client('cognito-identity', region_name=region)
    
    # Initiate the authentication request
    print('Logging In')
    authResult = cogIdp_client.initiate_auth(
        AuthFlow = 'USER_PASSWORD_AUTH',
        AuthParameters = {
            'USERNAME': username,
            'PASSWORD': password,
            },
        ClientId = clientID
        )
    # Get a cognito identity
    print('Getting ID')
    idResult = cog_client.get_id(
        IdentityPoolId = region + ':' + identityPoolID,
        Logins = {
            login: authResult['AuthenticationResult']['IdToken'],
            }
        )
    # Get credentials for the identity
    print('Getting Credentials')
    credResult = cog_client.get_credentials_for_identity(
        IdentityId = idResult['IdentityId'],
        Logins = {
            login: authResult['AuthenticationResult']['IdToken']
            }
        )
    gps_lambda_handler(credResult['Credentials'])       



#=====================================================
# Thread starter 
#=====================================================
if __name__ == '__main__':
    numArgs = len(sys.argv)

    # Kill all opening UDP port
    GPSHandler.killPathPredictionProgram()
    time.sleep(1)
        

    GPS_LogReadThread = Thread(target = GPSHandler.GPSLoggerSocketUDP, args=())
    GPS_LogReadThread.start()

    PathPredictionThread = Thread(target = pathPredictionThread, args=())
    PathPredictionThread.start()

    GPS_UDP_DistroThread = Thread(target=GPSHandler.GPSDistributorUDP, args=())
    GPS_UDP_DistroThread.start()

    pathPredThread = [[] for _ in range(GlobalVals.N_REAL_BALLOON)]
    for i in range(GlobalVals.N_REAL_BALLOON):
        pathPredThread[i] = Process(target=GPSHandler.startPathPredictionProgram,args=(i,))
        pathPredThread[i].start()

    try:
        cognito_login(GlobalVals.UNAME,GlobalVals.PWD)
    except(KeyboardInterrupt, SystemExit):
        print("Closing Program.")


    if GPS_LogReadThread.is_alive():
        with GlobalVals.BREAK_PLAYBACK_MUTEX:
            GlobalVals.BREAK_PLAYBACK = True
        GPS_LogReadThread.join()

    if PathPredictionThread.is_alive():
        with GlobalVals.BREAK_PLAYBACK_MUTEX:
            GlobalVals.BREAK_PLAYBACK = True
        with GlobalVals.BREAK_PATH_PREDICTION_THREAD_MUTEX:
            GlobalVals.BREAK_PATH_PREDICTION_THREAD = True
        PathPredictionThread.join()

    if GPS_UDP_DistroThread.is_alive():
        with GlobalVals.BREAK_PLAYBACK_MUTEX:
            GlobalVals.BREAK_PLAYBACK = True
        with GlobalVals.BREAK_GPS_UDP_DISTRO_MUTEX:
            GlobalVals.BREAK_GPS_UDP_DISTRO = True
        GPS_UDP_DistroThread.join()


    GPSHandler.killPathPredictionProgram()

        
        



