import numpy as np
    
def hover_trajectory(t = None):
    # Hover trajectory generator for a circle
    
    # =================== Your code goes here ===================
# You have to set the pos, vel, acc, yaw and yawdot variables
    
    # NOTE: the simulator will spawn the robot to be at the
#       position you return for t == 0
    pos = np.array([0, 0, 5])

    vel = np.array([0, 0, 0])
    acc = np.array([0, 0, 0])
    yaw = 0
    yawdot = 0
    # =================== Your code ends here ===================
    desired_state = {'x': pos, 'v': vel, 'x_ddot': acc, 'yaw': yaw, 'yaw_dot': yawdot}
    # desired_state.pos = pos
    # desired_state.vel = vel
    # desired_state.acc = acc
    # desired_state.yaw = yaw
    # desired_state.yawdot = yawdot
    # return desired_state
    
    return desired_state