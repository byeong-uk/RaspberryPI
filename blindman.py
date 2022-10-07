#blindman.py#
import serial               #import serial pacakge
from time import sleep
import webbrowser           #import package for opening link in browser
import sys                  #import system package
import RPi.GPIO as GPIO
import time
import socket
import threading

HOST = "172.20.10.2"
PORT = 8000
#GPIO Mode (BOARD / BCM)9
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24
GPIO_buzzer = 21

#Buzzer GPIO setting
GPIO.setup(GPIO_buzzer, GPIO.OUT)

#Ultrasonic GPIO setting
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

#TCP socket send thread
def send():
    global lat_in_degrees, long_in_degrees #global parameter lat,long
    while True:
        if lat_in_degrees is not "": #if data is fine
            data = str(lat_in_degrees)+"/"+str(long_in_degrees)
            print("Send DATA: ", data, "!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            s.sendall(data.encode()) #send
            time.sleep(5)
    s.close()
        
#TCP socket recv thread
def receive():
    while True:
        data = s.recv(1024)
        data = data.decode().strip()
        print("Recv DATA: ",data,"!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        if data == "danger": #if msg is danger buzzer on
            GPIO.output(GPIO_buzzer,True)
            time.sleep(0.5)
            GPIO.output(GPIO_buzzer,False)
            time.sleep(0.1)
    s.close()
 
def distance():# Ultrasonic sensor method
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
    
    if distance <= 25:#if dist is under 25cm buzzer on
        for _ in range(7):
            GPIO.output(GPIO_buzzer,True)
            time.sleep(0.05)
            GPIO.output(GPIO_buzzer,False)
            time.sleep(0.05)
    elif distance <= 50:
        for _ in range(5):
            GPIO.output(GPIO_buzzer,True)
            time.sleep(0.1)
            GPIO.output(GPIO_buzzer, False)
            time.sleep(0.1)
    elif distance <= 100:
        for _ in range(2):
            GPIO.output(GPIO_buzzer,True)
            time.sleep(0.1)
            GPIO.output(GPIO_buzzer, False)
            time.sleep(0.1)
    
    return distance

def Ultrasonic():
    while True:
        dist = distance()
        print ("Ultra sensor Measured Distance = %.1f cm" % dist)
        time.sleep(0.5)

def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
    nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
    
    #print("NMEA Time: ", nmea_time,'\n')
    #print ("NMEA Latitude:", nmea_latitude,"NMEA Longitude:", nmea_longitude,'\n')
    
    lat = float(nmea_latitude)                  #convert string into float for calculation
    longi = float(nmea_longitude)               #convertr string into float for calculation
    
    lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
    long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format
    
#convert raw NMEA string into degree decimal format   
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position

gpgga_info = "$GPGGA,"
ser = serial.Serial ("/dev/ttyS0")              #Open port with baud rate
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0

try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    sender = threading.Thread(target = send)
    receiver = threading.Thread(target = receive)
    ultra = threading.Thread(target = Ultrasonic)
    sender.start()
    receiver.start()
    ultra.start()

    while True:
        try:
            #Ultrasonic()
            received_data = (str)(ser.readline())                   #read NMEA string received
            #print(received_data)
            GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                 
            if (GPGGA_data_available>0):
                GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after "$GPGGA," string 
                NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
                GPS_Info()                                          #get time, latitude, longitude
 
                print("lat in degrees:", lat_in_degrees," long in degree: ", long_in_degrees, '\n')
                print("------------------------------------------------------------------------\n")
        except:
            continue
                                
except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()
    client_socket.close()
    sys.exit(0)
