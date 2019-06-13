import matplotlib.pyplot as plt
import serial

ser = serial.Serial("/dev/cu.usbserial-1430")
ser.baudrate = 9600
ser.timeout = 1

sensors = [0, 2]

sensor_data = [[] for sensor in range(len(sensors))]

if not ser.is_open:
    ser.open()

while len(sensor_data[0]) < 5:
    try:

        data = [int(i) for i in ser.readline().decode('utf-8').strip().split(",")]

        for data_store, sensor_num in enumerate(sensors):
            sensor_data[data_store].append(data[sensor_num])
    
    except:
        pass


point_counts = [i for i in range(len(sensor_data[0]))]

for i, sensor in enumerate(sensors):
    plt.plot(point_counts, sensor_data[i], label = "sensor " + str(sensor))

plt.legend(loc = 'upper right')
plt.xlabel('Point')
plt.ylabel('Takkstrip out')
# plt.ylim(0, 1024)
plt.show()
