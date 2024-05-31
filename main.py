#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from robot_control import RobotControl
from distance_control import DistanceControl
from lcd_control import LCDControl
from color_control import ColorControl
from state_manager import StateManager
from logger import Logger
from sound_light_control import SoundLightControl
from drive_manager import DriveManager, KP, KI, KD, ERROR_LIMIT
from kalman import Kalman

from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.parameters import Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.ev3devices import GyroSensor
import time
import math


from follower import Follower

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

DEFAULT_SPEED = 100 #120 #120 #100
DEFAULT_TURN_RATE = 0

# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
# ev3.speaker.beep()

control = RobotControl()
control.beep(ev3)
distance_control = DistanceControl()
color_control = ColorControl()
lcd_control = LCDControl(brick=ev3)
state_manager = StateManager(brick=ev3)
# logger = Logger('distance', 'color', 'timestamp')
# pid_logger1 = Logger('x', 'y', 'distance', 'model_angle', 'state_angle', name='_1pid_log')
# kalman_logger = Logger('x', 'y', 'distance', 'angle', name='_1klm_kalman_log')
# gyroscope_logger = Logger('x', 'y', 'distance', 'angle', name='_1klm_gyro_logs')

sound_light = SoundLightControl(ev3)
gyro_sensor = GyroSensor(Port.S4)

WHEEL_DIAMETER = 59 #59  #56 diamètre des roues en mm
AXLE_TRACK = 120 #119 #118 #117 #112 distance entre les roues en mm
'''Entre 17 et 18'''

left_motor = control.left_motor
right_motor = control.right_motor
# drive_base = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)
drive_base = DriveManager(left_motor, right_motor, color_control, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)
now = time.time()
future = now + 10
beep_num = 0
color = Color.BLACK
distance = 0
# control.rotate(-360*2, 500)
# ======================KP======================
errors = []
current_angle = 0
#TP2

# Facteur de correction du contrôleur PID en fonction de la pause de 0.1 seconde
CORRECTION_FACTOR = 1.5

# Calcul des paramètres du contrôleur PID corrigés
KP_corrected = KP * CORRECTION_FACTOR
KI_corrected = KI * CORRECTION_FACTOR
KD_corrected = KD * CORRECTION_FACTOR


# Variables pour l'odométrie
x_robot = 0
y_robot = 0
theta_robot = 0

x_state_coordinates = [0]
y_state_coordinates = [0]

x_kalman_coordinates = [0]
y_kalman_coordinates = [0]

x_gyroscope_coordinates = [0]
y_gyroscope_coordinates = [0]

last_time = time.time() #+ 0.2 # +0.2 pour que la première itération soit de 0.1
drive_base.reset()
last_distance = 0

kalman_filter = Kalman()
kalman_angle = 0
follower = Follower(left_motor, right_motor, name='Follower')
# while True:
#     distance = follower.get_distance()
#     print('Distance:', distance)
follower.follow()

# while True :#time.time() < future:
#     drive_base.drive()
    # make the robot move for 1 meter and stop


    ### TP2
    # lcd_control.write('%s : %s' % (color_control.mesure_error(), drive_base.angle()))
    
    # errors.append(color_control.mesure_error())
    # while KI*sum(errors) >= ERROR_LIMIT:
    #     errors = errors[100:]
    #     lcd_control.write('error limit reached')

    # if len(errors) > 0:
    #     current_angle = (KP*color_control.mesure_light()) + KI*sum(errors) + KD*(color_control.mesure_error() - errors[-1])
    # else:
    #     current_angle = (KP_corrected*color_control.mesure_light()) + KI_corrected*sum(errors)
    # # current_angle = 0
    # # drive_base.drive(DEFAULT_SPEED, current_angle)
    # drive_base.drive(DEFAULT_SPEED, current_angle)
    
    # gyro_angle = math.radians(gyro_sensor.angle())
    # model_angle = math.radians(current_angle)
    # kalman_angle = kalman_filter.filter(gyro_angle, model_angle)
    # print('Gyro angle:', gyro_angle, 'Model angle: ', model_angle, 'Kalman angle: ', kalman_angle)
  
    # # wait(100)
    

control.beep(ev3)
# control.stop()