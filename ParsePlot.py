import matplotlib.pyplot as plt
import serial

ser = serial.Serial("/dev/cu.usbserial-1430")
ser.baudrate = 115200
ser.timeout = 1

data = []

if not ser.is_open:
    ser.open()

while len(data) < 1000:
    try:
        data.append(ser.readline().decode('utf-8').strip())


    except:
        continue

point_counts = [i for i in range(len(data))]


plt.plot(point_counts, data)
plt.show()
