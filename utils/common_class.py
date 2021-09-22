import numpy as np 

#{RAW_QT: 1,2,3,4; MAGNETIC_VECTOR: 1,2,3; ACCELERATION: 1,2,3; EPOCH: 123456789; EULER_321: 1,2,3; MAG_HEADING: 123} 

class IMU:
    def __init__(self, sysID = None, raw_qt = None, mag_vector = None, accel = None, gyros = None, epoch = None, euler = None):
        self.sysID = sysID if sysID is not None else 0
        self.raw_qt = raw_qt if raw_qt is not None else np.array([[0.0, 0.0, 1.0, 0.0]]).T
        self.mag_vector = mag_vector if mag_vector is not None else np.zeros([3,1])
        self.accel = accel if accel is not None else np.array([[0.0, 0.0, -9.81]]).T
        self.gyros = gyros if gyros is not None else np.zeros([3,1])
        self.epoch = epoch if epoch is not None else 0.0
        self.euler = euler if euler is not None else np.zeros([3,1])

class IMU_PLAYBACK:
    def __init__(self, sysID = None, epoch = None, ax = None, ay = None, az = None, gx = None, gy = None, gz = None, mx = None, my = None, mz = None, qw = None, qx = None, qy = None, qz = None, roll = None, pitch = None, yaw = None):
        self.sysID = sysID if sysID is not None else 0
        self.epoch = epoch if epoch is not None else 0.0
        self.ax = ax if ax is not None else 0.0
        self.ay = ay if ay is not None else 0.0
        self.az = az if az is not None else 0.0
        self.gx = gx if gx is not None else 0.0
        self.gy = gy if gy is not None else 0.0
        self.gz = gz if gz is not None else 0.0
        self.mx = mx if mx is not None else 0.0
        self.my = my if my is not None else 0.0
        self.mz = mz if mz is not None else 0.0
        self.qw = qw if qw is not None else 0.0
        self.qx = qx if qx is not None else 0.0
        self.qy = qy if qy is not None else 0.0
        self.qz = qz if qz is not None else 0.0
        self.roll = roll if roll is not None else 0.0
        self.pitch = pitch if pitch is not None else 0.0
        self.yaw = yaw if yaw is not None else 0.0

class GPS:
    def __init__(self, sysID = None, lat = None, lon = None, alt = None, epoch = None, numSat = None):
        self.sysID = sysID if sysID is not None else 0
        self.lat = lat if lat is not None else 0.0
        self.lon = lon if lon is not None else 0.0
        self.alt = alt if alt is not None else 0.0
        self.epoch = epoch if epoch is not None else 0.0
        self.numSat = numSat if numSat is not None else 0

class BAROMETER:
    def __init__(self, sysID = None, alt = None, epoch = None):
        self.sysID = sysID if sysID is not None else 0
        self.alt = alt if alt is not None else 0.0
        self.epoch = epoch if epoch is not None else 0.0

class RSSI:
    def __init__(self, rssi_filtered = None, distance = None, epoch = None,targetPayloadID = None,sysID = None,distanceOnlineCalib=None,distanceGPS=None):
        self.rssi_filtered = rssi_filtered if rssi_filtered is not None else 0.
        self.distance = distance if distance is not None else 0.
        self.epoch = epoch if epoch is not None else 0.0
        self.targetPayloadID = targetPayloadID if targetPayloadID is not None else 0
        self.sysID = sysID if sysID is not None else 0
        self.distanceOnlineCalib = distanceOnlineCalib if distanceOnlineCalib is not None else 0.0
        self.distanceGPS = distanceGPS if distanceGPS is not None else 0.0
        
class POS_XYZ:
    def __init__(self, x = None, y = None, z = None):
        self.x = x if x is not None else 0.0
        self.y = y if y is not None else 0.0
        self.z = z if z is not None else 0.0

class TEMPERATURE:
    def __init__(self, sysID = None, temperature = None, epoch = None):
        self.sysID = sysID if sysID is not None else 0
        self.temperature = temperature if temperature is not None else 0.0
        self.epoch = epoch if epoch is not None else 0.0

class EKF:
    def __init__(self, sysID = None, lat = None, lon = None, alt = None, epoch = None, posX = None,posY = None, posZ = None, p00 = None, p01 = None, p10 = None, p11 = None, phase = None):
        self.sysID = sysID if sysID is not None else 0
        self.lat = lat if lat is not None else 0.0
        self.lon = lon if lon is not None else 0.0
        self.alt = alt if alt is not None else 0.0
        self.epoch = epoch if epoch is not None else 0.0
        self.posX = posX if posX is not None else 0.0
        self.posY = posY if posY is not None else 0.0
        self.posZ = posZ if posZ is not None else 0.0
        self.p00 = p00 if p00 is not None else 0.0
        self.p01 = p01 if p01 is not None else 0.0
        self.p10 = p10 if p10 is not None else 0.0
        self.p11 = p11 if p11 is not None else 0.0
        self.phase = phase if phase is not None else 0.0

# class LORA_ALLOCATION:
#     def __init__(self, pair1 = None, pair2 = None, pair3 = None):
#         self.pair1 = sysID if sysID is not None else 0
#         self.pair2 = temperature if temperature is not None else 0.0
#         self.pair3 = epoch if epoch is not None else 0.0

class PACKETSTATS:
    def __init__(self, sysID = None, packetStat = None, epoch = None):
        self.sysID = sysID if sysID is not None else 0
        self.packetStat = packetStat if packetStat is not None else 0.0
        self.epoch = epoch if epoch is not None else 0.0


