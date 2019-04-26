#!/usr/bin/env python

# ROS imports
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


# RPI imports
import pigpio
import time
import threading
from threading import Timer
import math
import signal
import sys

# Initialize RPI GPIO
pi = pigpio.pi()
if not pi.connected:
    raise IOError("Can't connect to pigpio")

# Constants
max_speed = 480
MAX_SPEED = max_speed

pin_M1DIAG = 5
pin_M2DIAG = 6
pin_M1PWM = 12
pin_M2PWM = 13
pin_M1EN = 22
pin_M2EN = 23
pin_M1DIR = 24
pin_M2DIR = 25
pin_ENCA = 16
pin_ENCB = 19


# Classes
class Motor(object):
    MAX_SPEED = max_speed
    def __init__(self, pwm_pin, dir_pin, en_pin, diag_pin):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.en_pin = en_pin
        self.diag_pin = diag_pin

        pi.set_pull_up_down(diag_pin, pigpio.PUD_UP) # make sure DIAG is pulled up
        pi.write(en_pin, 1) # enable driver by default

    def setSpeed(self, speed):
        if speed < 0:
            speed = -speed
            dir_value = 1
        else:
            dir_value = 0

        if speed > MAX_SPEED:
            speed = MAX_SPEED

        pi.write(self.dir_pin, dir_value)
        pi.hardware_PWM(self.pwm_pin, 20000, int(speed * 6250 / 3));
          # 20 kHz PWM, duty cycle in range 0-1000000 as expected by pigpio

    def enable(self):
        pi.write(self.en_pin, 1)

    def disable(self):
        pi.write(self.en_pin, 0)

    def getFault(self):
        return not pi.read(self.diag_pin)

class Motors(object):
    MAX_SPEED = max_speed

    def __init__(self):
        self.motor1 = Motor(pin_M1PWM, pin_M1DIR, pin_M1EN, pin_M1DIAG)
        self.motor2 = Motor(pin_M2PWM, pin_M2DIR, pin_M2EN, pin_M2DIAG)

    def setSpeeds(self, m1_speed, m2_speed):
        self.motor1.setSpeed(m1_speed)
        self.motor2.setSpeed(m2_speed)

    def enable(self):
        self.motor1.enable()
        self.motor2.enable()

    def disable(self):
        self.motor1.disable()
        self.motor2.disable()

    def getFaults(self):
        return self.motor1.getFault() or self.motor2.getFault()

    def forceStop(self):
        # reinitialize the pigpio interface in case we interrupted another command
        # (so this method works reliably when called from an exception handler)
        global pi
        pi.stop()
        pi = pigpio.pi()
        self.setSpeeds(0, 0)

class IntervalTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False


# Variables
motor = Motors()
countsteps = 0.0
oldtime = 0
newtime = 0
difftime = 0
vel = 0.0
tpr = 1200.0 # Ticks per rev (CPR/4)*GearRatio = (12/4)*100*4
dtime = 0
calcrunning = False

micros = lambda: int(round(time.time()))

class DriverFault(Exception):
    def __init__(self, driver_num):
        self.driver_num = driver_num

def raiseIfFault():
    if motors.motor1.getFault():
        raise DriverFault(1)
    if motors.motor2.getFault():
        raise DriverFault(2)

def encA_ISR(gpio, level, tick):
    print('ISR')
    if pi.read(pin_ENCB):
        countsteps += 1
    else:
        countsteps -= 1
    newtime = micros()
    difftime = newtime - oldtime
    oldtime = newtime

def calcVel(*args, **kwargs):
    if difftime != 0:
        vel = 1000000.0 / (tpr * difftime)
    dtime = difftime

# Start a timer running calcVel() every 10ms 
cv = IntervalTimer(0.01, calcVel, "World")

def publisher():
    rospy.init_node('motor_spin')
    pub = rospy.Publisher('/base_rotation', Pose, queue_size=10)
    rate = rospy.Rate(10)

    # Start Callback function (ISR)
    ISRA = pi.callback(pin_ENCA, pigpio.RISING_EDGE, encA_ISR)
    
    # Start Motors Spinning
    print('Enabling Motors')
    motor.enable()
    time.sleep(0.002)
    motor.setSpeeds(0, 0)
    time.sleep(0.002)
    motor.motor1.setSpeed(100)
    time.sleep(0.002)
    while not rospy.is_shutdown():
        theta = (countsteps % tpr) / (2*math.pi)
        p = Pose()
        p.position.x = 0.0
        p.position.y = 0.0
        p.position.z = 0.0
        # Make sure the quaternion is valid and normalized
        q = quaternion_from_euler(0.0, 0.0, theta)
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        pub.publish(p)
        rate.sleep()

def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        cv.stop()
        motor.setSpeeds(0, 0)
        motor.disable()

if __name__ == '__main__':
    try:
        publisher()
        signal.signal(signal.SIGINT, signal_handler)
    except rospy.ROSInterruptException:
        cv.stop()
        motor.setSpeeds(0, 0)
        motor.disable()
        pass