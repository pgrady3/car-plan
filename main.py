# Import Modules
import os, pygame
from pygame.locals import *
from pygame.compat import geterror
import numpy as np
import math
from shapely.geometry import Polygon
import random
import dubins
from queue import Queue, PriorityQueue

WIDTH = 800
HEIGHT = 800
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
turning_radius = 50.0
PRM_RAD = 400
PRM_PTS = 2000
OBS_QTY = 15
OBS_SIZE = 100

def ucs(prmlist, obstacles, start, end):
    """
    Function to compute UCS(Uniform Cost Search) for a graph
    :param graph: The graph to compute UCS for
    :param start: start node
    :param end: end node
    :param weights: A dictionary of weights; maps (start_node, end_node) -> weight
    """
    frontier = PriorityQueue()
    frontier.put((0, start))  # (priority, node)
    frontier.put((0.01, start.rotate180()))
    explored = set()
    bestEdges = []
    counter = 0

    endrotate = end.rotate180()
    prmlist.append(endrotate)

    while True:
        if frontier.empty():
            print("PRM FAIL!!!!!!")
            return None

        ucs_w, current_node = frontier.get()

        if current_node in explored:
            continue

        explored.add(current_node)

        if current_node == end or current_node == endrotate:
            break

        counter += 1
        if counter % 100 == 0:
            print(ucs_w, counter, len(explored), len(prmlist))

        for node in prmlist:
            if node in explored:
                continue

            if node.dist(current_node) > PRM_RAD:
                continue

            cost = current_node.pathCollide(node, obstacles)

            if cost == None:
                continue

            bestEdges.append((node, current_node))
            frontier.put((ucs_w + cost, node))

    #backtrack
    print('end', end)
    path = [end]
    while path[0] != start:
        for edge in bestEdges:
            if edge[0] == path[0]:
                path.insert(0, edge[1])
                print('from a to b', path[1], path[0])
                break

    print('start', start)

    return path



class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
     return '%.1f %.1f' % (self.x, self.y)

    def rot(self, theta):
        x = self.x * math.cos(theta) - self.y * math.sin(theta)
        y = self.y * math.cos(theta) + self.x * math.sin(theta)

        return Point(x, y)

    def offset(self, newpt):
        x = self.x + newpt.x
        y = self.y + newpt.y

        return Point(x, y)

    def drawtup(self):
        return (self.x, HEIGHT - self.y)

    def dist(self, pt):
        dx = self.x - pt.x
        dy = self.y - pt.y
        return math.sqrt(dx * dx + dy * dy)

class Rect(object):
    def __init__(self, pos, w, h, theta, color):
        self.pos = pos
        self.theta = theta
        self.w = w
        self.h = h
        self.color = color

        self.geom = [Point(-w/2, h/2), Point(w/2, h/2), Point(w/2, -h/2), Point(-w/2, -h/2)]

        self.pts = []
        for i in range(4):
            self.pts.append(self.geom[i].rot(self.theta).offset(self.pos))

        self.poly = None

        self.maxr = math.sqrt(self.w * self.w + self.h * self.h) / 2

    def __str__(self):
        return '%.1f %.1f %.1f' % (self.pos.x, self.pos.y, self.theta)

    def recompute(self):
        self.pts = []
        for i in range(4):
            self.pts.append(self.geom[i].rot(self.theta).offset(self.pos))

        self.poly = None

    def getPoints(self):
        return self.pts

    def draw(self, surface):
        color = self.color
        pts = self.getPoints()

        for i in range(4):
            p1 = pts[i]
            p2 = pts[(i + 1) % 4]

            pygame.draw.line(surface, color, p1.drawtup(), p2.drawtup(), 2)

    def getPoly(self):
        pts = self.pts

        if self.poly == None:#polygons are slow, make it only if needed
            self.poly = Polygon([pts[0].drawtup(), pts[1].drawtup(), pts[2].drawtup(), pts[3].drawtup()])

        return self.poly

    def collide(self, rect2):
        centerdist = self.pos.dist(rect2.pos)

        if centerdist > self.maxrad() + rect2.maxrad(): #quick coll check
            return False


        poly1 = self.getPoly()
        poly2 = rect2.getPoly()

        return poly1.intersects(poly2)

    def dist(self, rect2):
        return self.pos.dist(rect2.pos)

    def maxrad(self):
        return self.maxr

    def __lt__(self, other):
        return 0


class Car(Rect):
    def __init__(self, pos, theta):
        super(Car, self).__init__(pos, 50, 25, theta, BLACK)

    def path(self, end, step_size):
        path = dubins.shortest_path((self.pos.x, self.pos.y, self.theta), (end.pos.x, end.pos.y, end.theta), turning_radius)
        configurations, lens = path.sample_many(step_size)

        dubinsCars = []
        for c in configurations:
            dubinsCars.append(Car(Point(c[0], c[1]), c[2]))

        return dubinsCars

    def pathCollide(self, end, obstacles):
        path = dubins.shortest_path((self.pos.x, self.pos.y, self.theta), (end.pos.x, end.pos.y, end.theta), turning_radius)
        configurations, lens = path.sample_many(30)

        car = Car(Point(0, 0), 0)

        for cfg in configurations:
            car.pos.x = cfg[0]
            car.pos.y = cfg[1]
            car.theta = cfg[2]
            car.recompute()

            for obs in obstacles:
                if obs.collide(car):
                    return None

        return self.pathlen(end)

    def pathlen(self, end):
        path = dubins.shortest_path((self.pos.x, self.pos.y, self.theta), (end.pos.x, end.pos.y, end.theta), turning_radius)

        return path.path_length()

    def rotate180(self):
        return Car(self.pos, self.theta)


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((250, 250, 250))
    clock = pygame.time.Clock()
    screen.blit(background, (0, 0))
    pygame.display.flip()


    obstacles = []

    for i in range(OBS_QTY):
        pos = Point(np.random.uniform() * WIDTH, np.random.uniform() * HEIGHT)
        obstacles.append(Rect(pos, OBS_SIZE, OBS_SIZE, 0, BLUE))


    carPRM = []
    for i in range(PRM_PTS):
        pos = Point(np.random.uniform() * WIDTH, np.random.uniform() * HEIGHT)
        trycar = Car(pos, np.random.uniform() * np.pi)
        trycar.color = (200, 200, 200)

        coll = False
        for obs in obstacles:
            if obs.collide(trycar):
                coll = True

        if coll == False:
            carPRM.append(trycar)


    start = random.choice(carPRM)
    while start.pos.x > WIDTH * 0.25:
        start = random.choice(carPRM)

    end = random.choice(carPRM)
    while end.pos.x < WIDTH * 0.75:
        end = random.choice(carPRM)


    start.color = RED
    end.color = GREEN

    path = None

    going = True
    while going:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == QUIT:
                going = False
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                going = False
            
        screen.blit(background, (0, 0))

        for obs in obstacles:
            obs.draw(screen)

        for c in carPRM:
            c.draw(screen)

        if path is not None:
            for i in range(len(path) - 1):
                dc = path[i].path(path[i + 1], 5)

                for d in dc:
                    d.draw(screen)

        start.draw(screen)
        end.draw(screen)

        pygame.display.flip()


        if path is None:
            path = ucs(carPRM, obstacles, start, end)

    pygame.quit()


if __name__ == '__main__':
    main()
