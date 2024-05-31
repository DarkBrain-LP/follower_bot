from pybricks.ev3devices import UltrasonicSensor
from pybricks.parameters import Port

class DistanceControl:
    def __init__(self, port=Port.S2):
        self.ultrasonic = UltrasonicSensor(port)

    def distance(self):
        return self.ultrasonic.distance()
    
    def presence(self, distance=100):
        if self.distance() < distance:
            return True
    
    def handle_presence(self, distance=100):
        if self.presence(distance=distance):
            self.ultrasonic.lights.on(100)
        else:
            self.ultrasonic.lights.off()