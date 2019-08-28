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

WIDTH = 500
HEIGHT = 500
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
turning_radius = 30.0


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
    explored = []
    bestEdges = []

    while True:
        if frontier.empty():
            return None

        ucs_w, current_node = frontier.get()
        explored.append(current_node)

        print('ucs step')

        if current_node == end:
            break

        for node in prmlist:
            if node not in explored:
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

    def __str__(self):
        return '%.1f %.1f %.1f' % (self.pos.x, self.pos.y, self.theta)

    def getPoints(self):
        pts = []
        for i in range(4):
            pts.append(self.geom[i].rot(self.theta).offset(self.pos))

        return pts

    def draw(self, surface):
        color = self.color
        pts = self.getPoints()

        for i in range(4):
            p1 = pts[i]
            p2 = pts[(i + 1) % 4]

            pygame.draw.line(surface, color, p1.drawtup(), p2.drawtup(), 2)

    def getPoly(self):
        pts = self.getPoints()
        return Polygon([pts[0].drawtup(), pts[1].drawtup(), pts[2].drawtup(), pts[3].drawtup()])

    def collide(self, rect2):
        poly1 = self.getPoly()
        poly2 = rect2.getPoly()

        return poly1.intersects(poly2)

    def dist(self, rect2):
        return self.pos.dist(rect2.pos)


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
        dubinsCars = self.path(end, 20)

        for dc in dubinsCars:
            for obs in obstacles:
                if obs.collide(dc):
                    return None

        return self.pathlen(end)

    def pathlen(self, end):
        path = dubins.shortest_path((self.pos.x, self.pos.y, self.theta), (end.pos.x, end.pos.y, end.theta), turning_radius)

        return path.path_length()


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

    for i in range(10):
        pos = Point(np.random.uniform() * WIDTH, np.random.uniform() * HEIGHT)
        obstacles.append(Rect(pos, 50, 50, 0, BLUE))


    carPRM = []
    for i in range(50):
        pos = Point(np.random.uniform() * WIDTH, np.random.uniform() * HEIGHT)
        trycar = Car(pos, np.random.uniform() * np.pi)

        coll = False
        for obs in obstacles:
            if obs.collide(trycar):
                coll = True

        if coll == False:
            carPRM.append(trycar)


    start = random.choice(carPRM)
    start.color = RED
    end = random.choice(carPRM)
    end.color = GREEN

    path = ucs(carPRM, obstacles, start, end)


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

        for c in path:
            #c.color = (150, 150, 150)
            c.draw(screen)
        #print(path)


        for i in range(len(path) - 1):
            dc = path[i].path(path[i + 1], 5)

            for d in dc:
                d.draw(screen)


        start.draw(screen)
        end.draw(screen)

        pygame.display.flip()

    pygame.quit()


if __name__ == '__main__':
    main()
