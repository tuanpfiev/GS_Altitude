import sys
import numpy as np
sys.path.insert(1,'../utils/')
from navpy import lla2ned
from common_class import *
import math
import glob
import re
from datetime import datetime
import GlobalVariables
# def sysID_to_index(sysID):
#     if sysID == 1:
#         return 1
#     elif sysID == 2:
#         return 2
#     elif sysID == 253:
#         return 3
#     elif sysID == 254:
#         return 4
#     elif sysID == 255:
#         return 5
#     else:
#         print('SysID should be in the range 1-5')
#         os._exit(1)
#         return 0

def getFileList(parentFolder,fileName):
    fileList = []
    for file in glob.glob(parentFolder+fileName):
        fileList.append(file)
    return fileList

def getLatestFile(parentFolder,fileName):
    fileList = []
    lastestDate = datetime.strptime("19700101-000000","%Y%m%d-%H%M%S")
    lastestDateIndex= 0
    for file in glob.glob(parentFolder+"*"+fileName+"*"):
        fileList.append(file)

    fileName.replace("*","")
    
    for i in range(len(fileList)):
        fileDate = re.search(parentFolder+"(.+?)-"+fileName,fileList[i]).group(1)
        datetimeObject = datetime.strptime(fileDate, "%Y%m%d-%H%M%S")
        if datetimeObject > lastestDate:
            lastestDate = datetimeObject
            lastestDateIndex = i


    return fileList[lastestDateIndex],lastestDateIndex

def getLatestPrediction(parentFolder,fileName,balloonID):
    fileList = []
    lastestDate = 0
    lastestDateIndex= 0
    # for file in glob.glob(parentFolder+"*"+fileName+str(balloonID)+"*"):
    for file in glob.glob(parentFolder+"*"+fileName+str(balloonID)+"*"):
        fileList.append(file)

    if len(fileList)==0:
        return None,None

    fileName.replace("*","")
    
    for i in range(len(fileList)):
        fileDate = int(re.search(parentFolder+"(.+?)_"+fileName,fileList[i]).group(1))
        if fileDate > lastestDate:
            lastestDate = fileDate
            lastestDateIndex = i


    return fileList[lastestDateIndex],lastestDateIndex

def openTextFile(filePath,dataType):
    print("Open ",filePath," ...")
    dataArray = []
    with open(filePath) as f:
        # lines = f.read().splitlines()
        lines = f.read().splitlines()
        for i in range(len(lines)):
            eachLine = lines[i].split(',')
            try:
                if dataType == "GPS":
                    epoch   = float(eachLine[0])
                    sysID   = int(eachLine[1])
                    lon     = float(eachLine[2])
                    lat     = float(eachLine[3])
                    alt     = float(eachLine[4])
                    dataArray.append(GPS(sysID,lat,lon,alt,epoch))
                elif dataType == "EKF":
                    epoch = float(eachLine[0])
                    sysID   = int(eachLine[1])
                    lon     = float(eachLine[2])
                    lat     = float(eachLine[3])
                    alt     = float(eachLine[4])
                    dataArray.append(EKF(sysID,lat,lon,alt,epoch))

                elif dataType == "PacketStats":         
                    packetStats = [float(eachLine[0])]
                    for i in range(GlobalVariables.N_REAL_BALLOON):
                        packetStats.append(float(eachLine[len(eachLine)-GlobalVariables.N_REAL_BALLOON+i]))

                    dataArray.append(packetStats)
                elif dataType == "GPS_Sensor":
                    epoch   = float(eachLine[0])
                    sysID   = None
                    lon     = float(eachLine[1])
                    lat     = float(eachLine[2])
                    alt     = float(eachLine[3])
                    dataArray.append(GPS(sysID,lat,lon,alt,epoch))
                elif dataType == "IMU_Sensor":
                    epoch   = float(eachLine[1])
                    sysID   = None
                    ax      = float(eachLine[2])
                    ay      = float(eachLine[3])
                    az      = float(eachLine[4])
                    gx      = float(eachLine[5])
                    gy      = float(eachLine[6])
                    gz      = float(eachLine[7])
                    mx      = float(eachLine[8])
                    my      = float(eachLine[9])
                    mz      = float(eachLine[10])
                    qw      = float(eachLine[11])
                    qx      = float(eachLine[12])
                    qy      = float(eachLine[13])
                    qz      = float(eachLine[14])
                    roll    = float(eachLine[15])
                    pitch   = float(eachLine[16])
                    yaw     = float(eachLine[17])

                    dataArray.append(IMU_PLAYBACK(sysID,epoch,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,roll,pitch,yaw))
                else:
                    print("CHECK DATA TYPE IN READING TXT FILES")  
            except:
                pass              
        
    return dataArray

# def openTextFile(filePath,dataType):
#     print("Open ",filePath," ...")
#     dataArray = [[] for _ in range(GlobalVariables.N_REAL_BALLOON)]
#     with open(filePath) as f:
#         # lines = f.read().splitlines()
#         lines = f.read().splitlines()
#         for i in range(len(lines)):
#             eachLine = lines[i].split(',')
#             if dataType == "GPS":
#                 epoch   = float(eachLine[0])
#                 sysID   = int(eachLine[1])
#                 lon     = float(eachLine[2])
#                 lat     = float(eachLine[3])
#                 alt     = float(eachLine[4])
#                 dataArray[sysID-1].append(GPS(sysID,lat,lon,alt,epoch))
#             elif dataType == "EKF":
#                 epoch = float(eachLine[0])
#                 sysID   = int(eachLine[1])
#                 lon     = float(eachLine[2])
#                 lat     = float(eachLine[3])
#                 alt     = float(eachLine[4])
#                 dataArray[sysID-1].append(EKF(sysID,lat,lon,alt,epoch))

#             else:
#                 print("CHECK DATA TYPE IN READING TXT FILES")                
        
#     return dataArray

def sysID_to_index(sysID):
    if sysID == 1:
        return 1
    elif sysID == 2:
        return 2
    elif sysID == 3:
        return 3
    elif sysID == 253:
        return 4
    elif sysID == 254:
        return 5
    elif sysID == 255:
        return 6
    else:
        print('SysID should be in the range 1-6')
        os._exit(1)
        return 0

def extract_string_data(preString,endString,string_data):
    preIndex = string_data.find(preString)
    endIndex = string_data.find(endString, preIndex)
    return string_data[preIndex + len(preString):endIndex]

def convert_to_array(string_data):
    start_parsing = 0
    array = []
    while True:
        comma_index = string_data.find(",",start_parsing)
        if comma_index != -1:
            val = float(string_data[start_parsing:comma_index])
            array.append(val)
            start_parsing = comma_index + 1
        else:
            try:
                val = float(string_data[start_parsing:len(string_data)])
            except:
                break
            array.append(val)
            break
    return np.array(array).reshape(len(array),1)

def checkGPS(gps):
    if gps.lat == 0.0 or gps.lon == 0.0:
        return False
    else:
        return True
        
def checkAllGPS(gps_list):
    for i in range(len(gps_list)):
        if not checkGPS(gps_list[i]):
            return False
    return True

def checkRSSI(rssi):
    if rssi.epoch == 0.0 or rssi.distance == 0.0:
    # if rssi.epoch == 0.0:
        return False
    else:
        return True

def checkAllRSSI(rssi_list):
    if type(rssi_list[0]) == type(RSSI()):
        for i in range(len(rssi_list)):
            if not checkRSSI(rssi_list[i]):
                return False
        return True    
    else:
        for i in range(len(rssi_list)):
            for j in range(i+1,len(rssi_list[i])):
                if not checkRSSI(rssi_list[i][j]) or not checkRSSI(rssi_list[j][i]):
                    print(i,j,len(rssi_list[i]))
                    return False
        return True

def positionENU(gps,gps_ref):

    C_NED_ENU = np.array([[0,1, 0],[1,0,0],[0,0,-1]]) # correcting acc
    pos_ned = lla2ned(gps.lat, gps.lon, gps.alt, gps_ref.lat, gps_ref.lon, gps_ref.alt).reshape(3,1)
    pos_enu = np.dot(C_NED_ENU,pos_ned)
    return pos_enu


def distance2D(args):
    gps1 = args[0]
    gps2 = args[1]
    gps_ref = args[2]
    
    pos1_enu = positionENU(gps1,gps_ref)
    pos2_enu = positionENU(gps2,gps_ref)

    distance = np.array([math.sqrt((pos1_enu[0]-pos2_enu[0])**2+(pos1_enu[1]-pos2_enu[1])**2)])
    if distance == 0:
        return distance
    else:
        if len(args)==4:
            distance_rssi = args[3]
            if distance_rssi == 0:
                return distance
            else:
                distance = distance_rssi/np.linalg.norm(pos1_enu-pos2_enu) * distance
    
    return distance

def list_to_str(list_args):
    list_str = ""
    for i in range(len(list_args)):
        list_str = list_str + str(list_args[i]) + ","
    return list_str[:-1] + "\n"

#{|SYSTEM_ID: 1; EPOCH: 1615999399811; ACCELERATION: -0.237756,0.271728,9.774710; GYRO: 0.003350,-0.001536,0.004171; MAGNETIC_VECTOR: -0.068429,0.860688,0.786252; RAW_QT: 0.545915,0.010954,-0.008901,-0.837722; EULER_321: 1.539973,0.494716,-113.811455}
def stringToIMU(raw_data):
    try:
        raw_data.index("SYSTEM_ID:")
        raw_data.index("EPOCH:")
        raw_data.index("ACCELERATION:")
        raw_data.index("GYRO:")
        raw_data.index("MAGNETIC_VECTOR:")
        raw_data.index("RAW_QT:")
        raw_data.index("EULER_321:")

    except ValueError:
        
        return False, IMU()

    imu_i = IMU()
    try:
        imu_i.sysID = int(float(extract_string_data("SYSTEM_ID: ",";",raw_data)))
        imu_i.epoch = float(extract_string_data("EPOCH: ",";",raw_data))
        imu_i.accel = convert_to_array(extract_string_data("ACCELERATION: ",";",raw_data))
        imu_i.mag_vector = convert_to_array(extract_string_data("MAGNETIC_VECTOR: ",";",raw_data))
        imu_i.raw_qt = convert_to_array(extract_string_data("RAW_QT: ",";",raw_data))
        imu_i.euler = convert_to_array(extract_string_data("EULER_321: ",";",raw_data))
        imu_i.gyros = convert_to_array(extract_string_data("GYRO: ",";",raw_data))

        return True, imu_i

    except ValueError:

        return False, IMU()

def stringToGPS(raw_data):
    try:
        raw_data.index("'system':")
        raw_data.index("'altitude':")
        raw_data.index("'latitude':")
        raw_data.index("'longitude':")
        raw_data.index("'time':")

    except ValueError:
        
        return False, GPS()

    gps_i = GPS()

    try:
        gps_i.sysID = int(float(extract_string_data("'system': ",";",raw_data)))
        gps_i.alt = float(extract_string_data("'altitude': ",";",raw_data))
        gps_i.lat = float(extract_string_data("'latitude': ",";",raw_data))
        gps_i.lon = float(extract_string_data("'longitude': ",";",raw_data))
        gps_i.epoch = float(extract_string_data("'time': ","}",raw_data))

        return True, gps_i

    except ValueError:

        return False, GPS()

def stringToGPS_Manoeuvre(raw_data):
    try:
        raw_data.index("'system':")
        raw_data.index("'altitude':")
        raw_data.index("'latitude':")
        raw_data.index("'longitude':")
        raw_data.index("'time':")
        raw_data.index("'manoeuvre':")

    except ValueError:
        
        return False, GPS(), 0

    gps_i = GPS(1)

    try:
        gps_i.sysID = int(float(extract_string_data("'system':",";",raw_data)))
        gps_i.alt = float(extract_string_data("'altitude':",";",raw_data))
        gps_i.lat = float(extract_string_data("'latitude':",";",raw_data))
        gps_i.lon = float(extract_string_data("'longitude':",";",raw_data))
        gps_i.epoch = float(extract_string_data("'time':",";",raw_data))
        manoeuvre = float(extract_string_data("'manoeuvre':","}",raw_data))

        return True, gps_i,manoeuvre

    except ValueError:

        return False, GPS(), 0

def stringToBaro(raw_data):
    try:
        raw_data.index("SYSTEM_ID:")
        raw_data.index("ALTITUDE:")
        raw_data.index("EPOCH:")

    except ValueError:
        
        return False, BAROMETER()

    baro_i = BAROMETER()

    try:
        baro_i.sysID = int(float(extract_string_data("SYSTEM_ID: ",";",raw_data)))
        baro_i.epoch = float(extract_string_data("EPOCH: ",";",raw_data))
        baro_i.alt = float(extract_string_data("ALTITUDE: ",";",raw_data))
        return True, baro_i

    except ValueError:
        return False, BAROMETER()

def stringToRSSI(raw_data):
    try:
        raw_data.index("RSSI_filter:")
        raw_data.index("distance:")
        raw_data.index("time:")
        raw_data.index("targetPayloadID:")
        raw_data.index("sysID:")
        # raw_data.index("distanceOnlineCalib:")
        # raw_data.index("distanceGPS:")

    except ValueError:
        
        return False, RSSI()

    rssi_i = RSSI()

    try:
        rssi_i.rssi_filtered = float(extract_string_data("RSSI_filter: ",";",raw_data))
        rssi_i.distance = float(extract_string_data("distance: ",";",raw_data))
        rssi_i.epoch = float(extract_string_data("time: ",";",raw_data))
        rssi_i.targetPayloadID = int(float(extract_string_data("targetPayloadID: ",";",raw_data)))
        rssi_i.sysID = int(float(extract_string_data("sysID: ",";",raw_data)))
        # rssi_i.distanceOnlineCalib = float(extract_string_data("distanceOnlineCalib: ",";",raw_data))
        # rssi_i.distanceGPS = float(extract_string_data("distanceGPS: ",";",raw_data))

        # rssi_i.distance = 0
        return True, rssi_i

    except ValueError:

        return False, RSSI()

def stringToEKF(raw_data):
    try:
        raw_data.index("'system':")
        raw_data.index("'altitude':")
        raw_data.index("'latitude':")
        raw_data.index("'longitude':")
        raw_data.index("'time':")
        raw_data.index("'posX':")
        raw_data.index("'posY':")
        raw_data.index("'posZ':")
        raw_data.index("'p00':")
        raw_data.index("'p01':")
        raw_data.index("'p10':")
        raw_data.index("'p11':")
        raw_data.index("'phase':")

    except ValueError:
        
        return False, EKF()

    ekf_i = EKF()

    try:
        ekf_i.sysID = int(float(extract_string_data("'system': ",";",raw_data)))
        ekf_i.alt = float(extract_string_data("'altitude': ",";",raw_data))
        ekf_i.lat = float(extract_string_data("'latitude': ",";",raw_data))
        ekf_i.lon = float(extract_string_data("'longitude': ",";",raw_data))
        ekf_i.epoch = float(extract_string_data("'time': ",";",raw_data))
        ekf_i.posX = float(extract_string_data("'posX': ",";",raw_data))
        ekf_i.posY = float(extract_string_data("'posY': ",";",raw_data))
        ekf_i.posZ = float(extract_string_data("'posZ': ",";",raw_data))
        ekf_i.p00 = float(extract_string_data("'p00': ",";",raw_data))
        ekf_i.p01 = float(extract_string_data("'p01': ",";",raw_data))
        ekf_i.p10 = float(extract_string_data("'p10': ",";",raw_data))
        ekf_i.p11 = float(extract_string_data("'p11': ",";",raw_data))
        ekf_i.phase = float(extract_string_data("'phase': ",";",raw_data))

        return True, ekf_i

    except ValueError:

        return False, EKF()
        

def stringToTemperature(raw_data):
                        # socketPayload = "{'epoch': " + str(tempTime) + "; 'temp': " + str(tempVal) + ';}'

    try:
        raw_data.index("'system':")
        raw_data.index("'epoch':")
        raw_data.index("'temp':")

    except ValueError:
        
        return False, GPS()

    temp_i = TEMPERATURE()

    try:
        temp_i.sysID = int(float(extract_string_data("'system': ",";",raw_data)))
        temp_i.temperature = float(extract_string_data("'temp': ",";",raw_data))
        temp_i.epoch = float(extract_string_data("'epoch': ",";",raw_data))
        
        return True, temp_i

    except ValueError:

        return False, TEMPERATURE()        


def stringToLoraAllocation(raw_data):

    try:
        raw_data.index("'pair':")

    except ValueError:
        return False, 0
    
    temp_i = 0

    try:
        temp_i = int(float(extract_string_data("'pair': ",";",raw_data)))
        return True, temp_i

    except ValueError:
        return False, 0    

def stringToXP(raw_data):
    try:
        raw_data.index("'X00':")
        raw_data.index("'X01':")
        raw_data.index("'P00':")
        raw_data.index("'P01':")
        raw_data.index("'P10':")
        raw_data.index("'P11':")
    except ValueError:
        return False, 0
    
    XP = []
    try:
        X00 = float(extract_string_data("'X00': ",";",raw_data))
        X01 = float(extract_string_data("'X01': ",";",raw_data))
        P00 = float(extract_string_data("'P00': ",";",raw_data))
        P01 = float(extract_string_data("'P01': ",";",raw_data))
        P10 = float(extract_string_data("'P10': ",";",raw_data))
        P11 = float(extract_string_data("'P11': ",";",raw_data))
        return True, [X00, X01, P00, P01, P10, P11]
    except ValueError:
        return False, 0


def extract_str_btw_curly_brackets(data_str):
    string_list = []
    iterator = data_str.find('{')
    
    while data_str.find('}', iterator) != -1:
        substring_end = data_str.find('}', iterator)
        string_list.append(data_str[iterator:substring_end + 1])
        iterator = substring_end + 1
    return string_list


def valueInRange(val,valRange):

    if valRange[0] == None:
        if val > valRange[1]:
            return False
        else:
            return True
    elif valRange[1] == None:
        if val < valRange[0]:
            return False
        else:
            return True
    else:
        if val > valRange[1]:
            return False
        elif val < valRange[0]:
            return False
        else:
            return True


def getPhaseID_EKF(currentTime, timeArray, phaseID_Array):


    # print(currentTime-timeArray)
    if currentTime <= timeArray[0] or currentTime >= timeArray[-1]:
        # print(1)
        return 1
    
    for i in range(len(timeArray)):
        if currentTime < timeArray[i]:
            # print(phaseID_Array[i])
            return phaseID_Array[i]
            
    return 1

# Low-pass fileter 1/(tau*s+1)
def lowPassFilter(output,input,dt,tau):
    return (1-dt/tau)*output+dt/tau*input