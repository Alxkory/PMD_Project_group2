import gym
import env
##import trajectory
import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation


from controller import PDcontrolller
from hover import hover_trajectory
from circle import circle_trajectory
from diamond import diamond_trajectory
from tud import tud_trajectory
from tj_from_line import tj_from_line


env = gym.make('Quadrotor-v0')

current_state = env.reset(position=[0, 0, 0])

print("current:", current_state)
dt = 0.01
t = 0

track_tot = 0
bat_tot = 0

controller = PDcontrolller()
plan_trajectory = {'x': [], 'y': [], 'z': []}
real_trajectory = {'x': [], 'y': [], 'z': []}
while(t < 40):

    #desired_state = hover_trajectory(t)     # set while(t < 10):
    #desired_state = circle_trajectory(t)    # set while(t < 10):
    #desired_state = diamond_trajectory(t)   # set while(t < 8):
    desired_state = tud_trajectory(t)      # set while(t < 40):

    print("desired:", desired_state['x'])
    control_variable = controller.control(desired_state, current_state)

    action = control_variable['cmd_motor_speeds']
    print(action)
    obs, reward, done, info = env.step(action)
    print("--------------------------")
    print("current:", obs['x'])
    # real_trajectory.append(obs['x'])
    real_trajectory['x'].append(obs['x'][0])
    real_trajectory['y'].append(obs['x'][1])
    real_trajectory['z'].append(obs['x'][2])
    plan_trajectory['x'].append(desired_state['x'][0])
    plan_trajectory['y'].append(desired_state['x'][1])
    plan_trajectory['z'].append(desired_state['x'][2])
    current_state = obs
    track = (obs['x'][0]-desired_state['x'][0])**2 + (obs['x'][1]-desired_state['x'][1])**2 + (obs['x'][2]-desired_state['x'][2])**2
    track_tot = track_tot + track
    bat = action[0]**2 + action[1]**2 + action[2]**2 + action[3]**2
    bat_tot = bat_tot + bat
    t += dt

print('Tracking performance: ', track_tot)
print('Total time: ', t)
print('Total battery: ', bat_tot)

fig = plt.figure()
ax1 = p3.Axes3D(fig) # 3D place for drawing
real_trajectory['x'] = np.array(real_trajectory['x'])
real_trajectory['y'] = np.array(real_trajectory['y'])
real_trajectory['z'] = np.array(real_trajectory['z'])
plan_trajectory['x'] = np.array(plan_trajectory['x'])
plan_trajectory['y'] = np.array(plan_trajectory['y'])
plan_trajectory['z'] = np.array(plan_trajectory['z'])

point, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]], 'ro', label='Quadrotor')
line, = ax1.plot([plan_trajectory['x'][0]], [plan_trajectory['y'][0]], [plan_trajectory['z'][0]], label='Real_Trajectory')

ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
ax1.set_title('3D animate')
ax1.view_init(200, 200)
ax1.legend(loc='lower right')
ax1.set_xlim([-7, 7])
ax1.set_ylim([13, -7])
ax1.set_zlim([9, -2])
ax1.plot3D(plan_trajectory['x'],plan_trajectory['y'],plan_trajectory['z'], 'gray')

def animate(i):
    line.set_xdata(real_trajectory['x'][:i + 1])
    line.set_ydata(real_trajectory['y'][:i + 1])
    line.set_3d_properties(real_trajectory['z'][:i + 1])
    point.set_xdata(real_trajectory['x'][i])
    point.set_ydata(real_trajectory['y'][i])
    point.set_3d_properties(real_trajectory['z'][i])

ani = animation.FuncAnimation(fig=fig,
                              func=animate,
                              frames=len(real_trajectory['x']),
                              interval=30,
                              repeat=False,
                              blit=False)
plt.show()