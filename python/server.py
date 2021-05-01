import socket
import sys
from queue import Queue
from dataclasses import dataclass
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
from threading import Thread
import signal

running = True
lastmillis = 0
def serverloop():
    while running:
        recvbytes, recvaddr = s.recvfrom(1024)

        if not recvbytes: 
            break
        
        recvbytes.strip()

        teldata = TelemetryData.parse(recvbytes)
        
        global q
        q.put_nowait(teldata)
        print("server", q.qsize())

        curr = teldata.millis
        global lastmillis
        #print(curr - lastmillis)
        lastmillis = curr

@dataclass
class TelemetryData:
    millis: int
    bmp_temperature: float
    bmp_altitude: float
    bmp_pressure: float
    imu_accel_x: float
    imu_accel_y: float
    imu_accel_z: float
    imu_gyro_x: float
    imu_gyro_y: float
    imu_gyro_z: float
    imu_temperature: float

    def parse(bytesarr):
        d = bytesarr.split(b' ')
        return TelemetryData(
            int(d[0]),
            float(d[1]), float(d[2]), float(d[3]),
            float(d[4]), float(d[5]), float(d[6]),
            float(d[7]), float(d[8]), float(d[9]),
            float(d[10]))

class MyPlot:
    def __init__(self, titlename, titleunits, teldataattr):
        global win
        self.teldataattr = teldataattr
        self.plot = win.addPlot()
        self.plot.setLabel('top', titlename, units=titleunits)
        self.plot.setLabel('bottom', "time", units='ms')
        self.plot.showGrid(x=True, y=True)
        self.curve = self.plot.plot()
        self.datax = []
        self.datay = []

    def update(self, teldata):
        time = teldata.millis
        val = getattr(teldata, self.teldataattr)

        self.datax.append(time)
        self.datay.append(val)

        if len(self.datax) > 1000:
            self.datax = self.datax[-1000:]
            self.datay = self.datay[-1000:]
        
        self.curve.setData(x=self.datax, y=self.datay)

win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle('Rocket Telemetry')

imu_gyro_x = MyPlot("gyro X", "deg/s", "imu_gyro_x")
imu_gyro_y = MyPlot("gyro Y", "deg/s", "imu_gyro_y")
imu_gyro_z = MyPlot("gyro Z", "deg/s", "imu_gyro_z")
win.nextRow()
imu_accel_x = MyPlot("accel X", "m/s-1", "imu_accel_x")
imu_accel_y = MyPlot("accel Y", "m/s-1", "imu_accel_y")
imu_accel_z = MyPlot("accel Z", "m/s-1", "imu_accel_z")
win.nextRow()
bmp_altitude = MyPlot("bmp altitude", "m", "bmp_altitude")
bmp_pressure = MyPlot("bmp pressure", "kPa", "bmp_pressure")
win.nextRow()
imu_temperature = MyPlot("imu temperature", "celcius", "imu_temperature")
imu_temperature.plot.disableAutoRange("y")
imu_temperature.plot.setRange(yRange=(-20,40))
bmp_temperature = MyPlot("bmp temperature", "celcius", "bmp_temperature")
bmp_temperature.plot.disableAutoRange("y")
bmp_temperature.plot.setRange(yRange=(-20,40))

def update():
    global q

    print("update", q.qsize())
    while not q.empty():
        teldata = q.get()
        imu_gyro_x.update(teldata)
        imu_gyro_y.update(teldata)
        imu_gyro_z.update(teldata)
        imu_accel_x.update(teldata)
        imu_accel_y.update(teldata)
        imu_accel_z.update(teldata)
        bmp_altitude.update(teldata)
        bmp_pressure.update(teldata)
        imu_temperature.update(teldata)
        bmp_temperature.update(teldata)

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1000/10)

q = Queue()

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', 3333))

print('Server listening')

signal.signal(signal.SIGINT, signal.SIG_DFL)

t = Thread(target=serverloop)
t.start()

pg.mkQApp().exec_()
running = False
t.join()