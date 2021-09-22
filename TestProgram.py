import time
import socket
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
while True:
    if breakLoop:
        break
    epoch = time.time()
    send_data = "{'system': 1; 'latitude': -37; 'longitude': 142; 'altitude': 400; 'time':1630647918; 'manoeuvre': 1}"
    print(send_data)
    data_encoded = send_data.encode('utf-8')

    Distro_Connection.sendall(data_encoded)
    time.sleep(0.1)

s.close()
