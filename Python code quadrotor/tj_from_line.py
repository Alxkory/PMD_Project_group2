import numpy as np
    
def tj_from_line(start_pos = None,end_pos = None,time_ttl = None,t_c = None): 
    v_max = (end_pos - start_pos) * 2 / time_ttl
    if t_c >= 0 and t_c < time_ttl / 2:
        vel = v_max * t_c / (time_ttl / 2)
        pos = start_pos + t_c * vel / 2
        acc = np.array([[0],[0],[0]])
    else:
        vel = v_max * (time_ttl - t_c) / (time_ttl / 2)
        pos = end_pos - (time_ttl - t_c) * vel / 2
        acc = np.array([[0],[0],[0]])
    return pos,vel,acc