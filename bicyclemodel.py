#


# constants


# class definition
class KinematicBicycleModel():
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, throttle, delta):
        pass
