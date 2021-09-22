import time
import socket

import random
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1) 
s.bind(('127.0.0.1', 5001))
s.settimeout(100000)

try:
    s.listen(1) 
    Distro_Connection, addr = s.accept()  
    
    print("Logger Connected to ", addr)                                            
except Exception as e:
    print("Exception: " + str(e.__class__))
    print("Error in the GPSDistributor[",i,"] logger socket. Now closing thread.")
    
breakLoop = False
count = 0
while True:
    count = count + 1
    if breakLoop:
        break
    epoch = 1630647918 + count
    	

    lat = -36.7167 + count * 0.001
    lon = 142.2 + count * 0.001
    alt = 300 + count * 2
    send_data = "{'system': 1; 'latitude': " + str(lat) + "; 'longitude': " + str(lon) + "; 'altitude': " + str(alt) +"; 'time': " + str(epoch) + "; 'manoeuvre': " + str(random.choice(["-1", "0", "1"])) + "}"
    print(send_data)
    data_encoded = send_data.encode('utf-8')

    Distro_Connection.sendall(data_encoded)
    time.sleep(1)

s.close()
