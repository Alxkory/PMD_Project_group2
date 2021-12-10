import pygame
from RRTbasePy import RRTGraph
from RRTbasePy import RRTMap


def main():
    dimensions = (600, 1000)
    start = (50, 50)
    goal = (510, 510)
    obsdim = 30
    obsnum = 50
    iteration = 0

    pygame.init()
    map = RRTMap(start, goal, dimensions, obsdim, obsnum)
    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)

    obstacles = graph.makeobs()
    map.drawMap(obstacles)

    while (True):
        x, y = graph.sample_envir()
        n = graph.number_of_nodes()
        graph.add_node()
        graph.isFree()
        pygame.draw.circle(map.map, map.Red, (graph.x[n], graph.y[n]), map.nodeRad, map.nodeThickness)

    pygame.display.update()

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == '__main__':
    main()
