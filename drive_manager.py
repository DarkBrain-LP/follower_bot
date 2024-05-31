
# KP = 0.9#1.7
# KI = 0.3#0.15
# KD = 0.01#0.1
# KP = 0.9
# KI = 0.4
# KD = 0.01
# KP = 0.9
# KI = 0.07
# KD = 0.008
# KP = 0.6
# KI = 0.3
# KD = 0.02
# KP = 1.7
# KI = 0.15
# KD = 0.1
# x2
# KP = 1.5
# KI = 0.9
# KD = 0.207
from pybricks.robotics import DriveBase
from color_control import ColorControl
# KP = 1.7
# KI = 0.15
# KD = 0.1

# KP = 0.7
# KI = 0.15
# KD = 0.01

KP = 0.4 #1.1 #1.1 #0.8 #1.1 #1.3
KI = 0.3
KD = 0.1

# KP=2
# KI = 0.1
# KD = 0.005

DEFAULT_SPEED = 125
DEFAULT_TURN_RATE = 0
ERROR_LIMIT = 10
LEFT_ANGLE = 20
RIGHT_ANGLE = -20
# C droite B gauche couleur 3 distance 2

CORRECTION_FACTOR = 1.5

# Calcul des paramètres du contrôleur PID corrigés
KP_corrected = KP * CORRECTION_FACTOR
KI_corrected = KI * CORRECTION_FACTOR
KD_corrected = KD * CORRECTION_FACTOR

class DriveManager:
    def __init__(self, left_motor, right_motor, color_control:ColorControl, wheel_diameter=56, axle_track=112):
        self.drivebase = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=112)
        self.color_control = color_control
        self.errors = []
        self.current_angle = 0
        self.speed = DEFAULT_SPEED
    
    def drive(self, speed=None, turn_rate=None):
        if speed is not None and turn_rate is not None:
            self.drivebase.drive(speed, turn_rate)
            return
        
        if speed is not None:
            self.speed = speed
            
        if turn_rate is not None:
            self.current_angle = turn_rate
            self.drivebase.drive(self.speed, self.current_angle)
            return
        
        self.errors.append(self.color_control.mesure_error())
        while KI*sum(self.errors) >= ERROR_LIMIT:
            self.errors = self.errors[100:]
            # print('error limit reached')
        
        if len(self.errors) > 0:
            self.current_angle = (KP*self.color_control.mesure_light()) + KI*sum(self.errors) + KD*(self.color_control.mesure_error() - self.errors[-1])
        else:
            self.current_angle = (KP*self.color_control.mesure_light()) + KI*sum(self.errors)
        
        # self.drivebase.drive(speed, turn_rate)
        self.drivebase.drive(self.speed, self.current_angle)
        # print(self.current_angle)

    def reset(self):
        self.drivebase.reset()

    def stop(self):
        self.drivebase.stop()

    def bang_bang_kp(self, error, kp=1):
        self.drivebase.drive(error*kp, 0)

    def drive_bang_bang():
        if self.color_control.is_following_black_line():
            self.drivebase.drive(DEFAULT_SPEED, LEFT_ANGLE)
        elif self.color_control.is_following_white_line():
            self.drivebase.drive(DEFAULT_SPEED, RIGHT_ANGLE)

    def drive_kp():
        current_angle = abs(KP*self.color_control.mesure_light())
        if self.color_control.is_following_black_line():
            self.drivebase.drive(DEFAULT_SPEED, current_angle) 
        elif self.color_control.is_following_white_line():
            self.drivebase.drive(DEFAULT_SPEED, -current_angle) 

    def drive_pi():
        current_angle = abs(KP*self.color_control.mesure_light())
        errors.append(color_control.mesure_error())
        while KI*sum(errors) >= ERROR_LIMIT:
            errors = errors[100:]
            lcd_control.write('error limit reached')

        urrent_angle = (KP*color_control.mesure_light()) + KI*sum(errors)
        
        drive_base.drive(DEFAULT_SPEED, current_angle)
        logger.log(errors[-1])

    def drive_pid():
        current_angle = abs(KP*self.color_control.mesure_light())
        errors.append(color_control.mesure_error())
        while KI*sum(errors) >= ERROR_LIMIT:
            errors = errors[100:]
            lcd_control.write('error limit reached')

        if len(errors) > 0:
            current_angle = (KP*color_control.mesure_light()) + KI*sum(errors) + KD*(color_control.mesure_error() - errors[-1])
        else:
            current_angle = (KP*color_control.mesure_light()) + KI*sum(errors)
        
        drive_base.drive(DEFAULT_SPEED, current_angle)
        logger.log(errors[-1])

    def turn(self):
        angle = self.drivebase.angle()

        return self.drivebase.turn(angle)
    
    def turn_left(self, angle):
        pass
