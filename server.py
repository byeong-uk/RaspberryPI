#### server.py ####

import socket 
import threading
import time
import sys
 
def send():
    while True:
            data = "hello"
            conn.sendall(data.encode(UTF-8))
            time.sleep(10)
    conn.close()
        
def receive():
    while True:
        data = conn.recv(1024)
        data = data.decode(utf8).strip()
    conn.close()

HOST = ""
PORT = 7575

print("Socket Waiting!")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST,PORT))
s.listen(1)

conn, addr = s.accept()
print("Connected by "+addr)

receiver = threading.Thread(target = receive)
sender = threading.Thread(target = send)

sender.start()
receiver.start()
