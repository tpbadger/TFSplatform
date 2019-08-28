import serial
import time
import os
# Set serial port options
ser = serial.Serial("/dev/cu.usbserial-14340")
ser.baudrate = 9600
ser.timeout = 1

data_file = open('data.csv', 'w')

log = True

time.sleep(5) # wait for 5 secs to sync with port

while log:
    try:
        payload = ser.readline().decode('utf-8')
        data_file.write(payload)
    except:
        data_file.close()
        log = False

data_file.close()

new_fn = time.time()
current_dir = os.getcwd()

os.rename(current_dir + "/data.csv", current_dir + "/Data/data_" + str(new_fn) + ".csv")
