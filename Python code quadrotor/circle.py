import numpy as np
from tj_from_line import tj_from_line


def circle_trajectory(t):
    radius = 5
    dt = 0.01
    T = 10
    def pos_from_angle(a):
        pos = np.array([[radius*np.cos(a)],[radius*np.sin(a)],[2.5*a/(2*np.pi)]])
        return pos

    
    def get_vel(t):
        angle1 = tj_from_line(0,2 * np.pi,T,t)
        pos1 = pos_from_angle(angle1[0])
        angle2 = tj_from_line(0,2 * np.pi,T,t + dt)
        vel = (pos_from_angle(angle2[0]) - pos1) / dt
        return vel
    
    if t > T:
        pos = np.array([[0],[0],[2.5]])

        vel = np.array([[0],[0],[0]])

        acc = np.array([[0],[0],[0]])

    else:
        angle = tj_from_line(0, 2*np.pi, T, t)
        pos = np.array(pos_from_angle(angle[0]))
        pos[0] = pos[0] - radius
        pos = pos.T
        vel = np.array(get_vel(t))
        vel = vel.T
        acc = np.array((get_vel(t + dt) - get_vel(t)) / dt)
        acc = acc.T


    yaw = 0
    yawdot = 0
    # =================== Your code ends here ===================
    desired_state = {'x': pos[0], 'v': vel[0], 'x_ddot': acc[0], 'yaw': yaw, 'yaw_dot': yawdot}

    
    return desired_state