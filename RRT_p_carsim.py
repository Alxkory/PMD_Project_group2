# Main script. RRTbasePy.py contains the classes and methods used here.
import pygame
from task.RRTbasePy import RRTGraph
from task.RRTbasePy import RRTMap
import time
# from sys import exit
import bicyclemodel
import numpy as np
from bicyclemodel import KinematicBicycleModel
from bicyclemodel import TargetCourse
from bicyclemodel import pure_pursuit_steer_control
from bicyclemodel import PID


def main():
    dimensions = (600, 1000)
    start = (0,300)
    goal = (1000, 200)
    obsdim = 50
    obsnum = 20
    iteration = 0

    pygame.init()
    map = RRTMap(start, goal, dimensions, obsdim, obsnum)
    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)

    obstacles = graph.makeobs()
    map.drawMap(obstacles)

    while (not graph.path_to_goal()): # iterate until goal is found
        pygame.display.set_caption(f"planning path, iterations: {iteration}")
        if iteration % 10 == 0: # 10% of all steps i straight towards the goal
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey,
                               (X[-1], Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(
                map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness)
        else:
            X, Y, Parent = graph.expand()
            pygame.draw.circle(map.map, map.grey,
                               (X[-1], Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(
                map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness)
        if iteration % 5 == 0:
        # if iteration % 1 == 0:
            pygame.display.update()
            # time.sleep(0.15)
        iteration += 1
        
    
    
    # Extract path
    SmoothPath = graph.getSmoothPathCoords()
    map.drawPath(SmoothPath)
    print('smoothpath found, ready to drive')
    pygame.display.update()
    #pygame.event.clear()
    #pygame.event.wait(0)
    #time.sleep(5)
    #pygame.display.quit()
    #pygame.quit()
    #exit()
    print("setting targetcourse")

    smoothpath_array = np.array(SmoothPath)
    cx = (smoothpath_array.T)[0]
    cy = (smoothpath_array.T)[1]
    target_course = TargetCourse(cx, cy)
    course_points_np = np.array([cx,cy])
    course_points_pyg = (course_points_np.T).tolist()
    print("init car")
    car = KinematicBicycleModel(x=start[0],y=start[1],yaw=0.0)
    dt = bicyclemodel.dt

    target_speed = 100.0 / 3.6  # [m/s]

    time = 0.0
    T = 200
    clock = pygame.time.Clock()
    
    target_ind, _ = target_course.search_target_index(car)
    print("start simulation")
    simulation_running = True
    while simulation_running:
        for event in pygame.event.get():
                    if event.type == pygame.QUIT: 
                        running = False

        if T<=time:
            running = False

        Kp = 1

        steering, target_ind = pure_pursuit_steer_control(
            car, target_course, target_ind)

        throttle = PID(target_speed - car.v,Kp=1)
        
        car.update(throttle,steering)
        #print(f"running sim,sim-time: {time}")
        time += dt

        max_steer = car.max_steer
        steering = np.clip(steering,-max_steer,max_steer) #steering for drawing and title data, to match up with internal class delta

        #drawing
        #draw blanc
        map.map.fill((255, 255, 255, 255))
        # draw path
        map.drawPath(SmoothPath)
        # draw obstacles
        map.drawMap(obstacles)
        # draw target
        pygame.draw.circle(map.map, pygame.Color("green"), (cx[target_ind],cy[target_ind]), 3)
        #draw car
        car.draw_car_pygame(surface=map.map,delta=steering)
        # flip screen to get normal Y coordinate depiction
        #window.blit(pygame.transform.flip(window,False,True),(0,0))

        pygame.display.set_caption(f"x:{np.round(car.x, decimals=1)},y:{np.round(car.y, decimals=1)},yaw:{np.round(car.yaw, decimals=1)},v:{np.round(car.v, decimals=1)},delta:{np.round(steering, decimals=2)},time:{np.round(time, decimals=2)}")
        pygame.display.update()
        clock.tick(30)

    #while True:
    #    for event in pygame.event.get():
    #        if event.type == pygame.QUIT:
    #            pygame.quit()
    #            exit()


if __name__ == '__main__':
    # Sometimes the RRT algorithm raises an error. 
    # This exception handling makes the algoritm try again (until the error doesn't uccur)
    result=False
    while not result:
        try:
            main()
            print("main finished")
            result=True
        except:
            print('exeption occured')
            result=False