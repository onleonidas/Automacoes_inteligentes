import RPi.GPIO as GPIO
from pins import *
import time
from Encoder import Encoder
import ydlidar
import math
import numpy as np

class robo:

    def __init__(self):

        self.gpio = GPIO

        self.gpio.setmode(self.gpio.BCM)

        self.threshold = 0.05
        self.n_points = 25
        self.calibre = 2.1
        self.range = 3.14

        self.right_encoder = Encoder(RENCA, RENCB)
        self.right_encoder.start()
        self.left_encoder = Encoder(LENCA, LENCB)
        self.left_encoder.start()

        ydlidar.os_init()
        ports = ydlidar.lidarPortList()
        port = "/dev/ydlidar"
        for key, value in ports.items():
            port = value
            print(port)

        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 1.0)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, 3.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, 0.12)
        self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

        self.gpio.setup(RIN1, self.gpio.OUT)
        self.gpio.setup(RIN2, self.gpio.OUT)
        self.gpio.setup(RENA, self.gpio.OUT)
        self.gpio.output(RIN1, self.gpio.HIGH)
        self.gpio.output(RIN2, self.gpio.LOW)
        self.gpio.output(RENA, self.gpio.HIGH)

        self.gpio.setup(LIN1, self.gpio.OUT)
        self.gpio.setup(LIN2, self.gpio.OUT)
        self.gpio.setup(LENA, self.gpio.OUT)
        self.gpio.output(LIN1, self.gpio.HIGH)
        self.gpio.output(LIN2, self.gpio.LOW)
        self.gpio.output(LENA, self.gpio.HIGH)

        self.pwm_r = self.gpio.PWM(RENA, 1000)
        self.pwm_l = self.gpio.PWM(LENA, 1000)

        self.pwm_l.start(0)
        self.pwm_r.start(0)

        self.ret = self.laser.initialize()

        if self.ret:
            self.ret = self.laser.turnOn()
            self.scan = ydlidar.LaserScan()




    def interact(self):
        if self.ret and ydlidar.os_isOk():
            rv = 0
            lv = 0
            #Encoders
            if (self.right_encoder.new_data_flag()):
                rv = self.right_encoder.get_w()
                self.right_encoder.data_collected()
                if rv is None:
                    rv = 0

            if (self.left_encoder.new_data_flag() ):
                lv = -self.left_encoder.get_w()

                if lv is None:
                    lv = 0
                self.left_encoder.data_collected()

            lidar_points = self.get_lidar()
            return rv, lv, lidar_points

    def get_lidar(self):
        r = self.laser.doProcessSimple(self.scan)
        scan_data = []
        if r:

            for n in range(self.scan.points.size()):

                self.scan.points[n].angle += self.calibre

                if self.scan.points[n].range != 0:
                    angle_ = self.scan.points[n].angle
                    range_ = self.scan.points[n].range
                    time_ = self.scan.stamp

#                    print("antigo:", angle_)
                    angle_ = (angle_ + math.pi) % (2 * math.pi) - math.pi
#                    print("novo:" , angle_)
                    if (angle_ >= -self.range and angle_ <= self.range):
                        scan_data.append(self.makeRow(time_, range_, angle_))

            points = self.make_points(scan_data)
            return points
        return None

    def makeRow(self, stamp, distance, angle):
        row = {
            "SCAN_STAMP": stamp,
            "DISTANCE": distance,
            "ANGLE": angle
        }
        return row

    def make_points(self, df):
        alpha = 2 * self.range / (self.n_points - 1)
        enu_points = ["-165", "-150", "-135", "-120", "-105", "-90", "-75", "-60", "-45", "-30", "-15", "0",
                      "15", "30", "45", "60", "75", "90", "105", "120", "135", "150", "165", "180"]
        points = {
            "-165": 0,
            "-150": 0,
            "-135": 0,
            "-120": 0,
            "-105": 0,
            "-90": 0,
            "-75": 0,
            "-60": 0,
            "-45": 0,
            "-30": 0,
            "-15": 0,
            "0": 0,
            "15": 0,
            "30": 0,
            "45": 0,
            "60": 0,
            "75": 0,
            "90": 0,
            "105": 0,
            "120": 0,
            "135": 0,
            "150": 0,
            "165": 0,
            "180": 0
        }

        for i in range(self.n_points):
            p = list()
            angle = -self.range + i * alpha
            for j in df:
                if j["ANGLE"] >= angle - self.threshold and j["ANGLE"] <= angle + self.threshold:
#                    print(enu_points[i], j["DISTANCE"])
                    p.append(j["DISTANCE"])
            point = np.array(p).mean()
            try:
                points[enu_points[i]] = point
            except:
                continue
        return points

    def set_velocity(self, vl, vr,o_vl,o_vr):
        sinal_nl=1
        sinal_nr=1
        sinal_or=1
        sinal_ol=1

        if vl < 0:
            sinal_nl = -1
        if vr < 0:
            sinal_nr = -1
        if o_vl < 0:
            sinal_ol = -1
        if o_vr < 0:
            sinal_or = -1





        if sinal_nr != sinal_or:
            if vr < 0:
                vr = -vr
                self.gpio.output(RIN2, self.gpio.HIGH)
                self.gpio.output(RIN1, self.gpio.LOW)
            else:
                self.gpio.output(RIN1, self.gpio.HIGH)
                self.gpio.output(RIN2, self.gpio.LOW)
        else:
            if vr < 0:
                vr = -vr

        if sinal_nl != sinal_ol:
            if vl < 0:
                vl = -vl
                self.gpio.output(LIN2, self.gpio.HIGH)
                self.gpio.output(LIN1, self.gpio.LOW)
            else:
                self.gpio.output(LIN1, self.gpio.HIGH)
                self.gpio.output(LIN2, self.gpio.LOW)
        else:
            if vl < 0:
                vl = -vl


        self.pwm_l.start(vl)
        self.pwm_r.start(vr)

    def disconect(self):
        self.laser.turnOff()
        self.laser.disconnecting()
        self.gpio.cleanup()

