# This is the file that contains the classes and methods used in RRT.py
import random
import math
import pygame
import random
import numpy as np
from scipy import interpolate

random.seed()

# Pygame coordinate system:
# - origin in top left
# - positive x pointing right
# - positive y pointing down

# nodes are created on the go

class RRTMap: # for visualisation. Methods draw the map, obstacles and path
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        # setup variables
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions

        # window settings
        self.MapWindowName = 'RRT path planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw, self.Maph))
        self.map.fill((255, 255, 255, 255))
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obsdim = obsdim # dimensions of the obstacles
        self.obsNumber = obsnum # total number of obstacles

        # Different colors
        self.grey = (70, 70, 70) # RGB codes
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)

    def drawMap(self, obstacles):
        pygame.draw.circle(self.map, self.Green,
                           self.start, self.nodeRad + 5, 0)
        pygame.draw.circle(self.map, self.Green,
                           self.goal, self.nodeRad + 20, 1)
        self.drawObs(obstacles)

    def drawPath(self, path): # draws the path that has been found
        for node in path:
            pygame.draw.circle(self.map, self.Red, node, self.nodeRad, 0) # was self.nodeRad+3

    def drawObs(self, obstacles): # draws obstacles
        obstaclesList = obstacles.copy()
        while (len(obstaclesList) > 0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)


class RRTGraph: # this class contains the methods that provide the RRT functionality
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False # Set to True when goal is reached
        self.MapDimensions = MapDimensions
        self.maph, self.mapw = self.MapDimensions
        
        # lists to store nodes that are added to the tree:
        self.x = [] # list to store the x coordinates
        self.y = [] # list to store the y coordinates
        self.parent = [] # list to store the parent-IDs

        # initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        # the obstacles
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum

        # path
        self.goalstate = None
        self.path = []

    
    # The next 2 methods generate random obstacles:
    
    def makeRandomRect(self): # random x, y coordinates (upper left corner) | AANPASSEN
        uppercornerx = int(random.uniform(0, self.mapw - self.obsDim))
        uppercornery = int(random.uniform(0, self.maph - self.obsDim))
        return (uppercornerx, uppercornery)

    def makeobs(self): # creates the obstacles | AANPASSEN
        obs = []

        for i in range(0, self.obsNum):
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

    # The next 6 methods contain some node and edge tools
    def add_node(self, n, x, y): #stores a node in the lists. n = node index number
        self.x.insert(n, x)
        self.y.append(y)

    def remove_node(self, n): # removes a node from the lists.
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child): # adds an edge to the list 
        self.parent.insert(child, parent) # "child" is used as index, "parent" as element

    def remove_edge(self, n): # removes an edge from the list 
        self.parent.pop(n)

    def number_of_nodes(self): # returns total number of nodes
        return len(self.x)

    def distance(self, n1, n2): # returns distance between node n1 and node n2
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        dx = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (dx + py) ** (0.5)

    def sample_envir(self): # sample a random point, return its x and y coordinates
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self, n): # measures distance from new sampled node to every node in the tree
        dmin = self.distance(0, n)
        nnear = 0 # this will hold the ID of the closest node we found so far
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear # return the ID of the closest node

    def isFree(self): # Checks if node is in an obstacle. Returns False when it collides, Free when it's free.
        n = self.number_of_nodes() - 1 # node ID's start form 0
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            if rectang.collidepoint(x, y):
                self.remove_node(n) # remove node
                return False
        return True

    def crossObstacle(self, x1, x2, y1, y2): # Checks if an edge crosses an obstacle. Returns False when it collides, Free when it's free.
        obs = self.obstacles.copy()
        while (len(obs) > 0):
            rectang = obs.pop(0)
            for i in range(0, 101): # dit is een rechte lijn | STEERING FUNCTION INBOUWEN??????????????????
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                if rectang.collidepoint(x, y):
                    return True
        return False

    def connect(self, n1, n2):  # Connects 2 nodes and performs collision check
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2) # adds the edge to the tree when a connection is possible
            return True

    def step(self, nnear, n, dmax=35): # move node closer to tree when it's too far and check if goal has been reached (dmax = max distance from any node in the tree, default 35)
        d = self.distance(nnear, n)
        if d > dmax:
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xn, yn) = (self.x[n], self.y[n])
            (dx, dy) = (xn-xnear, yn-ynear)
            theta = math.atan2(dy, dx)
            (x, y) = (int(xnear+dmax*math.cos(theta)), # new node is created with a distance of
                      int(ynear+dmax*math.sin(theta))) # dmax to the closest node in the tree
            self.remove_node(n) # the node that is too far is deleted
            if abs(x-self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax: # check if the goal has been reached
                self.add_node(n, self.goal[0], self.goal[1])
                self.goalstate = n
                self.goalFlag = True
            else:
                self.add_node(n, x, y) # otherwise, simply add node to tree
                
                

    def path_to_goal(self): # If goal has not been reached, return False. If goal has been reached: creates a list containing the path (from goal to start)
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate) # starting at the goal
            newpos = self.parent[self.goalstate]
            while (newpos != 0): # adding the parent, the parent of that parent, etc.
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self): # Retrieve coordinates of the nodes in the path (to visualize it)
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords

    def B_spline(self):
        x=[]
        y=[]
        
        for point in self.path:
            x.append(self.x[point])
            y.append(self.y[point])
        
        tck, *rest = interpolate.splprep([x, y])
        u = np.linspace(0, 1, num=500)
        bspline=interpolate.splev(u, tck)
        # TODO: pas num aan
        
        return bspline

    def getSmoothPathCoords(self): # Retrieve coordinates of the nodes in the path (to visualize it)
        bspline = self.B_spline()
        SmoothPathCoords = []
        for i in range(len(bspline[0])):
            x, y = (bspline[0][i],bspline[1][i])
            SmoothPathCoords.append((x, y))
        return SmoothPathCoords


    def bias(self, ngoal): # Does a step straight towards the goal. This speeds up the algorithm when it is not used too much: now every 10th step (can be changed in RRT.py).
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self): # does a random expansion
        n = self.number_of_nodes()
        x, y = self.sample_envir() # sample a random point
        self.add_node(n, x, y) # add to list
        if self.isFree(): # if node is not in obstacle (else node gets removed inside isFree method)
            xnearest = self.nearest(n) # find nearest node
            self.step(xnearest, n) # move closer if it's too far
            self.connect(xnearest, n)
        return self.x, self.y, self.parent

    def cost(self):
        pass


