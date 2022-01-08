import numpy as np
from tj_from_line import tj_from_line
    
def diamond_trajectory(t = None):
    # DIAMOND trajectory generator for a diamond
    
    # =================== Your code goes here ===================
# You have to set the pos, vel, acc, yaw and yawdot variables
# NOTE: the simulator will spawn the robot to be at the
#       position you return for t == 0
    
    T = 8
    
    dt = 0.01
    
    
    def get_pos_vel(t):
        if t >= T:
            pos = np.array([[1],[0],[0]])
            vel = np.array([[0],[0],[0]])
            # description of the sides of the diamond
        elif t >= 0 and t < T * 0.25:
            pos,vel, acc1 = tj_from_line(np.array([[0],[0],[0]]),np.array([[0],[np.sqrt(2)],[np.sqrt(2)]]),T / 4,t)
        elif t >= T*0.25 and t < T * 0.5:
            pos,vel, acc1 = tj_from_line(np.array([[0],[np.sqrt(2)],[np.sqrt(2)]]),np.array([[0],[0],[2 * np.sqrt(2)]]), T/4,t - T / 4)
        elif t >= T / 2 and t < T * 0.75:
            pos,vel,acc1 = tj_from_line(np.array([[0],[0],[2 * np.sqrt(2)]]),np.array([[0],[- np.sqrt(2)],[np.sqrt(2)]]), T/4 ,t - T / 2)
        else:
            pos,vel,acc1 = tj_from_line(np.array([[0],[- np.sqrt(2)],[np.sqrt(2)]]),np.array([[1],[0],[0]]),T / 4,t - T * 3 / 4)
        return pos,vel
    
    if t >= T:
        # end position
        pos = np.array([[1],[0],[0]])
        vel = np.array([[0],[0],[0]])
        acc = np.array([[0],[0],[0]])
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
    # desired_state.pos = pos
    # desired_state.vel = vel
    # desired_state.acc = acc
    # desired_state.yaw = yaw
    # desired_state.yawdot = yawdot

    return desired_state