import numpy as np
import pygame

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

def PID(current_error,previous_error=0,integral_error=0,Kp=1,Ki=0,Kd=0):
    a = Kp * (current_error) + Kd*(previous_error) +Ki*(integral_error)
    return a

def purepursuit():

    throttle = 0
    steering = 0
    u = (throttle,steering)
    return u

def stanley():

    throttle = 0
    steering = 0
    u = (throttle,steering)
    return u

# constants
L = 3.0 #m
l_r = L/2.0
l_f = L-l_r
dt = 0.1
track = 2 #m
# class definition
class KinematicBicycleModel():
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0,max_steer=0.25*np.pi):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.max_steer = max_steer

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
        max_steer = self.max_steer
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

    def draw_car(self,plot=None,delta=0.0,drawvel=True):
        #draw body #xy position needs to be adjusted
        draw_rectangle(plot, L, 2, self.x, self.y, self.yaw,'red')
        #draw back wheels
        wheel_diam=0.8
        wheel_width=0.3
        plot.scatter(self.x,self.y,s=200,marker='D',color='black')
        b_wheel_x_r = (-l_r)
        b_wheel_y_r = (-track/2)
        b_wheel_r = (getRotation(self.yaw) @ np.array([[b_wheel_x_r],[b_wheel_y_r]])) + np.array([[self.x],[self.y]])
        #print(b_wheel_r)
        b_wheel_x_l = (-l_r)
        b_wheel_y_l = (track/2)
        b_wheel_l = (getRotation(self.yaw) @ np.array([[b_wheel_x_l],[b_wheel_y_l]])) + np.array([[self.x],[self.y]])
        #print(b_wheel_l)
        draw_rectangle(plot,wheel_diam ,wheel_width, b_wheel_r[0][0], b_wheel_r[1][0], self.yaw,'green')

        draw_rectangle(plot,wheel_diam, wheel_width, b_wheel_l[0][0], b_wheel_l[1][0], self.yaw,'green')
        #draw front wheels

        f_wheel_x_r = (l_f)
        f_wheel_y_r = (-track/2)
        f_wheel_r = (getRotation(self.yaw) @ np.array([[f_wheel_x_r],[f_wheel_y_r]])) + np.array([[self.x],[self.y]])
        #print(f_wheel_r)
        f_wheel_x_l = (l_f)
        f_wheel_y_l = (track/2)
        f_wheel_l = (getRotation(self.yaw) @ np.array([[f_wheel_x_l],[f_wheel_y_l]])) + np.array([[self.x],[self.y]])
        #print(f_wheel_l)
        draw_rectangle(plot, wheel_diam, wheel_width, f_wheel_l[0][0], f_wheel_l[1][0], self.yaw + delta,'orange')

        draw_rectangle(plot, wheel_diam, wheel_width, f_wheel_r[0][0], f_wheel_r[1][0], self.yaw + delta,'orange')

        if drawvel:
            plot.arrow(self.x,self.y,0.5*self.v*np.cos(self.yaw),0.5*self.v*np.sin(self.yaw),length_includes_head=True,hatch='|',ls='--')

    def draw_car_pygame(self,surface=None,delta=0.0):
        color_red = pygame.Color('red')
        color_black = pygame.Color('black')
        color_green = pygame.Color('green')
        color_orange = pygame.Color('orange')

        scaling_factor = 10

        scaled_x = self.x * scaling_factor
        scaled_y = self.y * scaling_factor

        #draw back wheels
        wheel_diam=0.8 #m
        wheel_width=0.3 #m

        scaled_wheel_diam = wheel_diam * scaling_factor
        scaled_wheel_width = wheel_width * scaling_factor

        scaled_L = L * scaling_factor
        scaled_track = track * scaling_factor

        scaled_l_r = l_f * scaling_factor
        scaled_l_f = l_f * scaling_factor

        #draw body #xy position needs to be adjusted
        draw_rectangle_pygame(surface, scaled_L, scaled_track, scaled_x, scaled_y, self.yaw, color_red)# draw rectangular polygon
        ...#draw coordinate of car as point back
        b_wheel_x_r = (-scaled_l_r)
        b_wheel_y_r = (-scaled_track/2)
        b_wheel_r = (getRotation(self.yaw) @ np.array([[b_wheel_x_r],[b_wheel_y_r]])) + np.array([[scaled_x],[scaled_y]])

        b_wheel_x_l = (-scaled_l_r)
        b_wheel_y_l = (scaled_track/2)
        b_wheel_l = (getRotation(self.yaw) @ np.array([[b_wheel_x_l],[b_wheel_y_l]])) + np.array([[scaled_x],[scaled_y]])

        draw_rectangle_pygame(surface,scaled_wheel_diam, scaled_wheel_width, b_wheel_r[0][0], b_wheel_r[1][0], self.yaw,color_green)

        draw_rectangle_pygame(surface,scaled_wheel_diam, scaled_wheel_width, b_wheel_l[0][0], b_wheel_l[1][0], self.yaw,color_green)
        #draw front wheels

        f_wheel_x_r = (scaled_l_f)
        f_wheel_y_r = (-scaled_track/2)
        f_wheel_r = (getRotation(self.yaw) @ np.array([[f_wheel_x_r],[f_wheel_y_r]])) + np.array([[scaled_x],[scaled_y]])

        f_wheel_x_l = (scaled_l_f)
        f_wheel_y_l = (scaled_track/2)
        f_wheel_l = (getRotation(self.yaw) @ np.array([[f_wheel_x_l],[f_wheel_y_l]])) + np.array([[scaled_x],[scaled_y]])

        draw_rectangle_pygame(surface, scaled_wheel_diam, scaled_wheel_width, f_wheel_l[0][0], f_wheel_l[1][0], self.yaw + delta,color_orange)

        draw_rectangle_pygame(surface, scaled_wheel_diam, scaled_wheel_width, f_wheel_r[0][0], f_wheel_r[1][0], self.yaw + delta,color_orange)
        center_radius = scaling_factor * 0.3
        pygame.draw.circle(surface,color_black,(scaled_x,scaled_y),center_radius)

def draw_rectangle(plot,width,height,Xcenter,Ycenter,Yaw,color):
    xycorners = np.array([[width/2,-width/2,-width/2,width/2,width/2],     #x
                         [height/2,height/2,-height/2,-height/2,height/2]])#y

    xycorners = getRotation(Yaw) @ xycorners
    centerarray = np.array([[Xcenter,Xcenter,Xcenter,Xcenter,Xcenter],
                             [Ycenter,Ycenter,Ycenter,Ycenter,Ycenter]])
    xycorners = xycorners + centerarray

    plot.plot(xycorners[0],xycorners[1],color=color)
    return None

def draw_rectangle_pygame(surface,width,height,Xcenter,Ycenter,Yaw,color):
    xycorners = np.array([[width/2,-width/2,-width/2,width/2,width/2],     #x
                         [height/2,height/2,-height/2,-height/2,height/2]])#y

    xycorners = getRotation(Yaw) @ xycorners
    centerarray = np.array([[Xcenter,Xcenter,Xcenter,Xcenter,Xcenter],
                             [Ycenter,Ycenter,Ycenter,Ycenter,Ycenter]])
    xycorners = xycorners + centerarray

    points = (xycorners.T).tolist()
    pygame.draw.polygon(surface,color,points)
    return None

def normalizeAngle(angle):
    while angle > np.pi:
        angle -= 2.0*np.pi

    while angle < -np.pi:
        angle += 2.0*np.pi

    return angle

def getRotation(angle):
    matrix = np.array([[np.cos(angle),-np.sin(angle)],
                       [np.sin(angle),np.cos(angle)]])        

    return matrix