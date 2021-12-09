import random
import math
import pygame

class RRTMap:
    def __init__(self, start, goal, MapDimension, obsdim, obsnum ):
        #setup variables
        self.start = start
        self.goal = goal
        self.MapDimension = MapDimension
        self.Maph, self.Mapw = self.MapDimension

        #window settings
        self.MapWindowName = 'RRT path planning'
        pygame.display. set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw,self.Maph))
        self.map.fill((255, 255, 255, 255))
        self.nodeRad = 0
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        #Different colors
        self.grey = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)



    def drawMap(self):
        pygame.draw.circle(self.map, self.Green, self.start, self.nodeRad+5,0)
        pygame.draw.circle(self.map, self.Green, self.goal, self.nodeRad+20,1)
        self.drawObs((obstacles))

    def drawPath(self):
        pass

    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while (len(obstaclesList)>0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(selfmap, self.grey, obstacle)


class RRTGraph:
     def __init__(self, start, goal, MapDimension, obsdim, obsnum ):
         (x,y) = start
         self.start = start
         self.goal = goal
         self.goalFlag = False
         self.Maph, self.Mapw = self.MapDimension
         self.x = []
         self.y = []
         self.parent = []

         #initialize the tree
         self.x.append(x)
         self.y.append(y)
         self.parent.append(0)

         #the obstacles
         self.obstacles = []
         self.obsDim = obsdim
         self.obsNum = obsnum

         #path
         self.goalstate = None
         self.path = []

     def makeRandomRect(self):
         uppercornerx = int(random.uniform(0, self.Mapw - self.obsDim))
         uppercornery = int(random.uniform(0, self.Maph - self.obsDim))

         return (uppercornerx, uppercornery)

     def makeobs(self):
         obs = []

         for i in range(0, self.obsNum)
             rectang = None
             startgoalcol = True
             while startgoalcol:
                 upper = self.makeRandomRect()
                 rectang = pygame.Rect(upper, (self.obsDim, self.obsDim))
                 if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                     startgoalcol = True
                 else:
                     startgoalcol = False
                 obs.append(rectang)
             self.obstacles = obs.copy()
             return obs

     def add_node(self):
         pass

     def remove_node(self):
         pass

     def add_edge(self):
         pass

     def remove_edge(self):
         pass

     def number_of_nodes(self):
         pass

     def distance(self):
         pass

     def nearest(self):
         pass

     def isFree(self):
         pass

     def crossObstacle(self):
         pass

     def connect(self):
         pass

     def step(self):
         pass

     def path_to_goal(self):
         pass

     def getPathCoords(self):
         pass

     def bias(self):
         pass

     def expand(self):
         pass

     def cost(self):
         pass

