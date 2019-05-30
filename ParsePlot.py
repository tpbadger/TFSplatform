import matplotlib.pyplot as plt
import serial
import numpy as np

ser = serial.Serial("/dev/cu.usbserial-1430")
ser.baudrate = 9600
ser.timeout = 1

if not ser.is_open:
    ser.open()



sensors = ('Sensor 1', 'Sensor 2', 'Sensor 3', 'Sensor 4', 'Sensor 5')
y_pos = np.arange(len(sensors))
sensor_data = np.zeros(5, dtype=int)


ax.bar(y_pos, sensor_data, 'b')

plt.show()



def update_plot():
    ax.bar(y_pos, sensor_data, 'b')
    fig.canvas.draw()



for _ in range(0, 1):
    try:

        data = ser.readline().decode('utf-8').strip()

        sensor_data = [int(num) for num in data.split(',')]


    except:

        pass
        print('broke')

    update_plot()









