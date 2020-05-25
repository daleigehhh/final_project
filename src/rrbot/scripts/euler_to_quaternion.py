#!/usr/bin/env python

from math import *
class RPYToQuaternion:

    roll = input("roll: ")
    pitch = input("pitch: ")
    yaw = input("yaw: ")

    def get_x(self):
        print sin(self.roll*pi/360)*cos(self.yaw*pi/360)*cos(self.pitch*pi/360) - cos(self.roll*pi/360)*sin(self.yaw*pi/360)*sin(self.pitch*pi/360)
    def get_y(self):
        print cos(self.roll*pi/360)*sin(self.yaw*pi/360)*cos(self.pitch*pi/360) + sin(self.roll*pi/360)*cos(self.yaw*pi/360)*sin(self.pitch*pi/360)
    def get_z(self):
        print cos(self.roll*pi/360)*cos(self.yaw*pi/360)*sin(self.pitch*pi/360) - sin(self.roll*pi/360)*sin(self.yaw*pi/360)*cos(self.pitch*pi/360)
    def get_w(self):
        print cos(self.roll*pi/360)*cos(self.yaw*pi/360)*cos(self.pitch*pi/360) + sin(self.roll*pi/360)*sin(self.yaw*pi/360)*sin(self.pitch*pi/360)

if __name__ == "__main__":
    RPYToQuaternion().get_x()
    RPYToQuaternion().get_y()
    RPYToQuaternion().get_z()
    RPYToQuaternion().get_w()