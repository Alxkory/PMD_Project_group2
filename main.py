# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import bicyclemodel
import numpy as np
import matplotlib.pyplot as plt
from bicyclemodel import KinematicBicycleModel

def main():
    # Use a breakpoint in the code line below to debug your script.


    car = KinematicBicycleModel(x=0.0,y=0.0,yaw=0.2)
    dt = bicyclemodel.dt

    time = 0.0
    T = 300

    output_euler = []

    while T>=time:
        
        

        if car.v < 10:
            throttle = 1
        else:
            throttle = 0

        steering = np.sin(time*0.1)*1
        
        car.update(throttle,steering,method="RK4")
        time += dt
        if True:
            plt.cla()
            plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            max_steer = car.max_steer
            steering = np.clip(steering,-max_steer,max_steer)
            car.draw_car(plot=plt,delta=steering)
            plt.axis("equal")
            plt.grid(True)

            plt.xlim([car.x-5,car.x+5])
            plt.ylim([car.y-5,car.y+5])

            plt.title(f"x:{np.round(car.x,decimals=1)},y:{np.round(car.y,decimals=1)},yaw:{np.round(car.yaw,decimals=1)},v:{np.round(car.v,decimals=1)},delta:{np.round(steering,decimals=2)},time:{np.round(time,decimals=2)}")
            plt.pause(0.001)

    if True:
        plt.cla()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        plt.grid(True)
        plt.show()
  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
