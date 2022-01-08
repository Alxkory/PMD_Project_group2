import numpy as np
from tj_from_line import tj_from_line

def tud_trajectory(t):
    T = 40

    dt = 0.01

    def pos_from_angle(a):
        pos = np.array([[radius*np.cos(a)],[radius*np.sin(a)],[2.5*a/(2*np.pi)]])
        return pos

    def get_pos_vel(t):
        if t >= T:
            pos = np.array([[0], [8], [7]])
            vel = np.array([[0], [0], [0]])
            # description of the sides of the diamond
        elif t >= 0 and t < T * 0.1:
            pos, vel, acc1 = tj_from_line(np.array([[0], [0], [0]]), np.array([[0], [0], [7]]), T / 10, t)
        elif t >= T * 0.1 and t < T * 0.2:
            pos, vel, acc1 = tj_from_line(np.array([[0], [0], [7]]),
                                          np.array([[0], [-2], [7]]), T / 10, t - T / 10)
        elif t >= T *0.2 and t < T * 0.3:
            pos, vel, acc1 = tj_from_line(np.array([[0], [-2], [7]]),
                                          np.array([[0], [3], [7]]), T / 10, t - T / 5)
        elif t >= T *0.3 and t < T * 0.4:
            pos, vel, acc1 = tj_from_line(np.array([[0], [3], [7]]),
                                          np.array([[0], [3], [2]]), T / 10, t - T * 3/ 10)
        elif t >= T *0.4 and t < T * 0.5:
              #curve U
              pos1 = 0
              pos2 = 3 + (4/(T/10))*(t-(T*0.4))
              pos3 = -np.sqrt(4-(pos2-5)**2)+2
              pos = np.array([[pos1], [pos2], [pos3]])
              vel1 = 0
              vel2 = (4/(T/10))
              vel3  = (80*(20*t-9*T))/(np.sqrt(-(1600*t**2-1440*t*T+320*T**2)/(T**2))*T**2)
              vel = np.array([[vel1], [vel2], [vel3]])
        elif t >= T *0.5 and t < T * 0.6:
            pos, vel, acc1 = tj_from_line(np.array([[0], [7], [2]]),
                                          np.array([[0], [7], [7]]), T / 10, t - T / 2)
        elif t >= T *0.6 and t < T * 0.7:
            pos, vel, acc1 = tj_from_line(np.array([[0], [7], [7]]),
                                          np.array([[0], [8.5], [7]]), T / 10, t - T * 3/ 5)
        elif t >= T *0.7 and t < T * 0.8:
             #curve D
             pos1 = 0
             pos3 = 7-(7 /(T/10))*(t - (T * 0.7))
             pos2 = np.sqrt(3.5*3.5 - (pos3-3.5)**2) +8.5
             pos = np.array([[pos1], [pos2], [pos3]])

             vel1 = 0
             vel3 = (7 / (T / 10))
             vel2 = -(1225*t*(3*T-4*t))/((np.sqrt(-(2744*T**2-7350*T*t+4900*t**2)/T**2)*T**3))
             vel  = np.array([[vel1], [vel2], [vel3]])


        elif t >= T *0.8 and t < T * 0.9:
            pos, vel, acc1 = tj_from_line(np.array([[0], [8.5], [0]]),
                                          np.array([[0], [8], [0]]), T / 10, t - T *0.8)
        else:
            pos, vel, acc1 = tj_from_line(np.array([[0], [8], [0]]), np.array([[0], [8], [7]]), T / 10,
                                          t - T * 9 / 10)
        return pos, vel


    if t >= T:
        # end position
        pos = np.array([[0], [8], [7]])
        vel = np.array([[0], [0], [0]])
        acc = np.array([[0], [0], [0]])
    else:
        # travel position, velocity and accaleration
        pos, vel = get_pos_vel(t)
        pos1, vel1 = get_pos_vel(t + dt)

        acc = (vel1 - vel) / dt

    pos = pos.T
    vel = vel.T
    acc = acc.T


    yaw = 0
    yawdot = 0
    # =================== Your code ends here ===================
    desired_state = {'x': pos[0], 'v': vel[0], 'x_ddot': acc[0], 'yaw': yaw, 'yaw_dot': yawdot}
    return desired_state