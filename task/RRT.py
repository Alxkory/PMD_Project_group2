# Main script. RRTbasePy.py contains the classes and methods used here.
import pygame
from RRTbasePy import RRTGraph
from RRTbasePy import RRTMap
import time


def main():
    dimensions = (600, 1000)
    start = (50, 50)
    goal = (510, 510)
    obsdim = 50
    obsnum = 20
    iteration = 0

    pygame.init()
    map = RRTMap(start, goal, dimensions, obsdim, obsnum)
    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)

    obstacles = graph.makeobs()
    map.drawMap(obstacles)

    while (not graph.path_to_goal()): # iterate until goal is found
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

    map.drawPath(graph.getPathCoords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait()
    # time.sleep(5)


if __name__ == '__main__':
    # sometimes the RRT algorithm raises an error. 
    # This exception handling makes the algoritm try again (until the error doesn't uccur)
    result=False
    while not result:
        try:
            main()
            result=True
        except:
            result=False