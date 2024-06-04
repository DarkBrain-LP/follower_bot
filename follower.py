from distance_control import DistanceControl
from drive_manager import DriveManager
from logger import Logger
from robot_control import RobotControl
from color_control import ColorControl
import time
Ts = 500
D = 150
a = 0.5
Vmax = 240
Tl = 500
af = 0.5
aa = 0.5
Df = 150
Da = 300

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
        self.speed = 0
        # distance_logger = Logger('leader', 'follower', 'timestamp', name='_log_distance')
        # velocity_logger = Logger('vitesse_follower', 'Ts', 'Df', 'Da', 'af', 'aa', name='_log_velocity')
        # self.distance_logger = Logger('distance', 'timestamp', name='_log_distance')
        # self.velocity_logger = Logger('vitesse_follower', 'Ts', 'Df', 'Da', 'af', 'aa', name='_log_velocity')

        # logger for follow_one_point
        self.distance_logger_one_point = Logger('distance', 'timestamp', name='_log_distance_one_point')
        self.velocity_logger_one_point = Logger('vitesse_follower', 'Ts', 'D', 'a', name='_log_velocity_one_point')

        # logger for follow_two_points
        self.distance_logger_two_points = Logger('distance', 'timestamp', name='_log_distance_two_points')
        self.velocity_logger_two_points = Logger('vitesse_follower', 'Ts', 'Df', 'Da', 'af', 'aa', name='_log_velocity_two_points')



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
        last_time = time.time()
        distance = self.distance_sensor.distance()
        self.speed = max(min(50, a * (distance - D)), 0)
        self.drivebase.drive(speed=self.speed)

        while True:
            # apply control every Ts
            now = time.time()
            # print('Now:', now, 'Last time:', last_time, 'Ts=', Ts, 'now - last_time >= Ts/1000 ?', now - last_time >= Ts/1000)
            # print('now - last_time=', now - last_time, 'Ts/1000=', Ts/1000)
            if now - last_time >= Ts/1000:
                print('condition verified')
                distance = self.distance_sensor.distance()
                self.speed = max(min(50, a * (distance - D)), 0)
                last_time = now
                self.drivebase.drive(speed=self.speed)
                print('Distance:', distance, 'Next speed:', self.speed)
                self.distance_logger_one_point.log(distance, time.time())
                self.velocity_logger_one_point.log(self.speed, Ts, D, a)



    def follow_two_points(self):
        distance = self.distance_sensor.distance()
        # je veux calculer la vitesse de freinage à un point F et la vitesse d'accélération à un point A
        # On calcule ensuite la vitesse à appliquer de la manière suivante:
        # vitesse = min (%𝑓(𝑡 + 𝑇𝑠), %𝑎(𝑡 + 𝑇𝑠))
        # implémente cette politique
        # point F
        v_a = min(max(aa * (distance - Da), 0, self.speed), 50)
        v_f = min(max(af * (distance - Df), 0), 50)
        update_acceleration = True if v_f < v_a else False
        
        self.speed = min(v_f, v_a)
        
        self.drivebase.drive(speed=self.speed)

        last_time = time.time()
        distance = self.distance_sensor.distance()

        while True:
            # apply control every Ts
            if time.time() - last_time >= Ts/1000:
                distance = self.distance_sensor.distance()
                if update_acceleration:
                    v_a = min(max(aa * (distance - Da), 0, self.speed), 50)
                    update_acceleration = False
                else:
                    v_f = min(max(af * (distance - Df), 0), 50)
                    update_acceleration = True

                self.speed = min(v_f, v_a)
                self.drivebase.drive(speed=self.speed)
                last_time = time.time()
                print('Distance:', distance, 'Next speed:', self.speed)
                self.distance_logger_two_points.log(distance, time.time())
                self.velocity_logger_two_points.log(self.speed,Ts, Df, Da, af, aa, time.time())


    