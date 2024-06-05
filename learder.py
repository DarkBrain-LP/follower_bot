import time
from distance_control import DistanceControl
from drive_manager import DriveManager
from robot_control import RobotControl
from color_control import ColorControl
Ts = 500
D = 150
a = 0.5
Vmax = 240

class Leader():
    def __init__(self, left_motor, right_motor, name='Leader'):
        self.name = name
        self.max_velocity = 100
        self.control = RobotControl()
        self.color_control = ColorControl()
        self.drivebase = DriveManager(self.control.left_motor, self.control.right_motor, self.color_control)

    
    def leader(self):
        while True:
            start_time = time.time()
            while time.time() - start_time < 5:
                print("Leader advancing")
                self.drivebase.drive()
                time.sleep(0.1)  
            print("Leader stop")
            self.drivebase.stop() 
            time.sleep(5) 
            
            
            
           
    


    