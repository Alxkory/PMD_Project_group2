import numpy as np

def euler(func,y,dt):
    y_new = y + dt*func(y)
    return y_new

def RK4(func,y,dt):
    #print(f"inside rk4, func:{func}")
    #print(f"inside rk4, y:{y}")
    #print(f"result of func in rk4: {func(y)}")
    k1 = dt * func(y)
    #print(f"rk4, k1:{k1}")
    k2 = dt * func(y + 0.5 * k1)
    k3 = dt * func(y + 0.5 * k2)
    k4 = dt * func(y + k3)
    y_new = y + (1 / 6) * (k1 + k2 + k3 + k4)
    return y_new

# constants
L = 3.0 #m
l_r = L/2.0
l_f = L-l_r
dt = 0.1

# class definition
class KinematicBicycleModel():
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def vehiclekinematics(self,s):
        x,y,psi,v,throttle,steering = s

        beta = np.arctan((l_r/(l_f+l_r))*np.tan(steering))
        dx = v*np.cos(psi+beta)
        dy = v*np.sin(psi+beta)
        dpsi = (v/l_r)*np.sin(beta)
        dv = throttle
        w = np.array([dx, dy, dpsi,dv,throttle,steering])
        return w

    def update(self, throttle, delta,method="RK4"):
        # clips input
        max_steer = 0.25*np.pi
        delta = np.clip(delta,-max_steer,max_steer)
        #sets input array
        s = (self.x,self.y,self.yaw,self.v,throttle,delta)
        # runs dynamics and interpolates for new state
        if method == "RK4":
            s_new = RK4(self.vehiclekinematics,s,dt)
        if method == "euler":
            s_new = euler(self.vehiclekinematics,s,dt)
        # retreives new state
        self.x,self.y,self.yaw,self.v,_,_ = s_new
        # ensures yaw angle remains bound
        self.yaw = normalizeAngle(self.yaw)



def normalizeAngle(angle):
    while angle > np.pi:
        angle -= 2.0*np.pi

    while angle < -np.pi:
        angle += 2.0*np.pi

    return angle