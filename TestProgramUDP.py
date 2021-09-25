import time
import socket

import random
s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

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

    s.sendto(data_encoded,('127.0.0.1',5001))


    time.sleep(1)

s.close()
