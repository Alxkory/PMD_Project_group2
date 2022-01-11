# Main script. RRTbasePy.py contains the classes and methods used here.
import pygame
from task.RRTbasePy import RRTGraph
from task.RRTbasePy import RRTMap
import time as tm
# from sys import exit
import bicyclemodel
import numpy as np
from bicyclemodel import KinematicBicycleModel
from bicyclemodel import TargetCourse
from bicyclemodel import pure_pursuit_steer_control
from bicyclemodel import PID
import pandas as pd

def RRT_p_simulation(rectangle_inflation):
    dimensions = (600, 1000)
    start = (30,300)
    goal = (970, 200)
    obsdim = 50
    obsnum = 20
    iteration = 0

    pygame.init()
    starttime = tm.time()
    map = RRTMap(start, goal, dimensions, obsdim, obsnum,rectangle_inflation)
    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum,rectangle_inflation)

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
            #tm.sleep(0.001)
        iteration += 1
    endtime = tm.time()
    RRT_solution_time = endtime - starttime
    num_iter = iteration
    
    # Extract path
    SmoothPath = graph.getSmoothPathCoords()
    map.drawPath(SmoothPath)
    print('smoothpath found, ready to drive')
    pygame.display.update()
    #pygame.event.clear()
    #pygame.event.wait(0)
    #tm.sleep(20)
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

    delta_x = cx[1] - cx[0]
    delta_y = cy[1] - cy[0]
    yaw = np.arctan2(delta_y, delta_x)

    print("init car")
    car = KinematicBicycleModel(x=start[0],y=start[1],yaw=yaw)
    dt = bicyclemodel.dt

    target_speed = 100.0 / 3.6  # [m/s]

    time = 0.0
    T = 200
    #clock = pygame.time.Clock()
    
    target_ind, _ = target_course.search_target_index(car)
    print("start simulation")
    simulation_running = True
    goal_reached = False
    collision_occured = False
    out_of_time = False

    while simulation_running:
        for event in pygame.event.get():
                    if event.type == pygame.QUIT: 
                        running = False

        if time > T:
            print("out of time")
            out_of_time = True
            simulation_running = False

        Kp = 1
        
        dist_to_goal = np.hypot(goal[0]-car.x,goal[1]-car.y) 
        if dist_to_goal < 20:
            print('goal reached')
            goal_reached = True

        if goal_reached:
            simulation_running = False

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
        collision_bool = car.draw_car_pygame(surface=map.map,delta=steering,obstacles=obstacles)
        # flip screen to get normal Y coordinate depiction
        #window.blit(pygame.transform.flip(window,False,True),(0,0))
        if collision_bool:
            collision_occured = True

        pygame.display.set_caption(f"x:{np.round(car.x, decimals=1)},y:{np.round(car.y, decimals=1)},yaw:{np.round(car.yaw, decimals=1)},v:{np.round(car.v, decimals=1)},delta:{np.round(steering, decimals=2)},time:{np.round(time, decimals=2)}")
        pygame.display.update()
        #clock.tick(30)

    pygame.display.update()
    #tm.sleep(5)

    #while True:
    #    for event in pygame.event.get():
    #        if event.type == pygame.QUIT:
    #            pygame.quit()
    #            exit()
    return num_iter,RRT_solution_time,collision_occured,out_of_time


if __name__ == '__main__':
    num_sims = 100

    simulation_number_list = []
    rectangle_inflation_param_list = []
    number_of_iterations_list = []
    RRT_solution_time_list = []
    collision_occured_list = []
    planner_successful_list = []
    out_of_time_list = []
    for i in range(num_sims):
        rectangle_inflation = 10*(i % 7)
        num_iter = None
        pathplanner_successful = None
        RRT_solution_time = None
        collision_occured = None
        out_of_time = False
        try:
            num_iter,RRT_solution_time,collision_occured,out_of_time = RRT_p_simulation(rectangle_inflation)
            pathplanner_successful = True
        except:
            print('solution failed')
            pathplanner_successful = False

        simulation_number_list.append(i)
        rectangle_inflation_param_list.append(rectangle_inflation)
        number_of_iterations_list.append(num_iter)
        RRT_solution_time_list.append(RRT_solution_time)
        collision_occured_list.append(collision_occured)
        planner_successful_list.append(pathplanner_successful)
        out_of_time_list.append(out_of_time)
    df = pd.DataFrame(
        {
            "simulation_number": simulation_number_list,
            "rectangle_inflation_param": rectangle_inflation_param_list,
            "number_of_iterations":  number_of_iterations_list,
            "RRT_solution_time": RRT_solution_time_list,
            "collision_occured": collision_occured_list,
            "planner_successful": planner_successful_list,
            "out_of_time": out_of_time_list
        }
    
    )
    save_to_pickle = False    
    if save_to_pickle:
        df.to_pickle("rrt_+_simulation_results.pkl")
    
    
    # Sometimes the RRT algorithm raises an error. 
    # This exception handling makes the algoritm try again (until the error doesn't uccur)
    #result=False
    #while not result:
    #    try:
    #        main()
    #        print("main finished")
    #        result=True
    #   except:
    #        print('exeption occured')
    #       result=False