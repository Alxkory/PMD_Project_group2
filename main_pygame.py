# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import bicyclemodel
import numpy as np
from bicyclemodel import KinematicBicycleModel
from bicyclemodel import TargetCourse
from bicyclemodel import pure_pursuit_steer_control
import sys
import pygame
from pygame.locals import *
#sys.path.append("/task/")
from task.RRTbasePy import RRTGraph
from task.RRTbasePy import RRTMap

def main():
    # Use a breakpoint in the code line below to debug your script.
    pygame.init()
    W = 1000
    H = 600
    window = pygame.display.set_mode((W,H))
    
    start_x = 100
    start_y = 300
    start_yaw = np.deg2rad(-60)

    goal_x = 900
    goal_y = 0

    #set target course
    cx = np.arange(start_x, goal_x, 0.1) #placeholders
    cy = [(start_y + 100*np.sin(ix / 30.0)) for ix in cx] #placeholders
    #for visualisation
    course_points_np = np.array([cx,cy])
    course_points_pyg = (course_points_np.T).tolist()
    #init target course class
    target_course = TargetCourse(cx, cy)
    # set desired speed
    target_speed = 100.0 / 3.6  # [m/s]
    #init vehicle
    car = KinematicBicycleModel(x=start_x,y=start_y,yaw=start_yaw)
    #get dt to match with internal dt of car class, should be switched later on
    dt = bicyclemodel.dt
    #set initial time, set maximum time
    time = 0.0
    T = 100
    clock = pygame.time.Clock()

    target_ind, _ = target_course.search_target_index(car)

    running=True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: 
                running = False
                
        if T<=time:
            running = False
            
        if car.v < target_speed:
            throttle = 1
        elif car.v > target_speed:
            throttle = -0.1
        else:
            throttle = 0

        steering, target_ind = pure_pursuit_steer_control(
            car, target_course, target_ind)

        car.update(throttle,steering)
        time += dt

        max_steer = car.max_steer
        steering = np.clip(steering,-max_steer,max_steer) #steering for drawing and title data, to match up with internal class delta

        #drawing
        #draw blanc
        window.fill((255, 255, 255, 255))
        # draw path
        for node in course_points_pyg:
            pygame.draw.circle(window, pygame.Color('black'), node, 1)
        # draw target
        pygame.draw.circle(window, pygame.Color("green"), (cx[target_ind],cy[target_ind]), 3)
        #draw car
        car.draw_car_pygame(surface=window,delta=steering)
        # flip screen to get normal Y coordinate depiction
        window.blit(pygame.transform.flip(window,False,True),(0,0))

        pygame.display.set_caption(f"x:{np.round(car.x, decimals=1)},y:{np.round(car.y, decimals=1)},yaw:{np.round(car.yaw, decimals=1)},v:{np.round(car.v, decimals=1)},delta:{np.round(steering, decimals=2)},time:{np.round(time, decimals=2)}")
        pygame.display.update()
        clock.tick(30)

    pygame.event.clear()
    pygame.event.wait()


  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/