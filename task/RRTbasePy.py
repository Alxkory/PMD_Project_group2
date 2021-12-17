# This is the file that contains the classes and methods used in RRT.py
import random
import math
import pygame

# Pygame coordinate system:
# - origin in top left
# - positive x pointing right
# - positive y pointing down


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

    def drawPath(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.Red, node, self.nodeRad+3, 0)

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
        self.goalFlag = False
        self.MapDimensions = MapDimensions
        self.maph, self.mapw = self.MapDimensions
        
        # lists to store nodes (that are added to the tree):
        self.x = [] # list to store the x coordinate of nodes (of the tree)
        self.y = [] # list to store the y coordinate of nodes (of the tree)
        self.parent = [] # list to store the parent-id of nodes of the tree

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
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (px + py) ** (0.5)

    def sample_envir(self): # sample a random point, return its x and y coordinates
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self, n):  # finds nearest node
        dmin = self.distance(0, n)
        nnear = 0  # this will hold the ID of the closest node we found so far
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def isFree(self): # Checks if node is in an obstacle. Returns False when it collides, Free when it's free.
        n = self.number_of_nodes() - 1 # node ID's start form 0
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            if rectang.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def crossObstacle(self, x1, x2, y1, y2): # Checks if an adge crosses an obstacle. Returns False when it collides, Free when it's free.
        obs = self.obstacles.copy()
        while (len(obs) > 0):
            rectang = obs.pop(0)
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                if rectang.collidepoint(x, y):
                    return True
        return False

    def connect(self, n1, n2): # Perfoms collision check for edge between nodes n1 and n2. WWWWWWWWWWWWWWWWWWW
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax=35):
        d = self.distance(nnear, nrand)
        if d > dmax:
            u = dmax/d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand-xnear, yrand-ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear+dmax*math.cos(theta)),
                      int(ynear+dmax*math.sin(theta)))
            self.remove_node(nrand)
            # check if the goal has been reached
            if abs(x-self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords

    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent

    def cost(self):
        pass
