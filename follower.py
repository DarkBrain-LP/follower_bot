from distance_control import DistanceControl
from drive_manager import DriveManager
from robot_control import RobotControl
from color_control import ColorControl
Ts = 500
D = 150
a = 0.5
Vmax = 240

class Follower():
    def __init__(self, left_motor, right_motor, name='Follower'):
        self.name = name
        self.distance_sensor = DistanceControl()
        self.max_velocity = 100
        self.distace_threshold = D
        self.control = RobotControl()
        self.color_control = ColorControl()
        self.drivebase = DriveManager(self.control.left_motor, self.control.right_motor, self.color_control)
        self.stop = False
        self.next_speed = 0


    def get_distance(self):
        return self.distance_sensor.distance()
    
    def follow(self):
        while True:
            distance = self.distance_sensor.distance()
            if distance < self.distace_threshold:
                self.stop = True
                print('Stop')
                self.drivebase.stop()
            else:
                self.drivebase.drive(speed=Vmax/2)
    
    def follow_one_point(self):
        distance = self.distance_sensor.distance()
        self.speed = max(min(self.max_velocity, self.next_speed + a * (distance - D)), 0)
        if distance < self.distace_threshold:
            self.stop = True
            print('Stop')
            self.drivebase.stop()
        else:    
            self.drivebase.drive()

    