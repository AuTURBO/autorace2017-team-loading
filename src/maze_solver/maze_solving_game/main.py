import cv2
import numpy as np
import datetime
from world import World
from obstacle_creator import ObstacleCreator
from solver import Solver





start = [7, 7]
goal = [4,4]
size = [10,10]

obstacle_creator = ObstacleCreator(size)
solver = Solver(size, start, goal, obstacle_creator.map)

while (True):
    print 'new cycle', datetime.datetime.now()
    solver.solve_distance()
    print 'solver end', datetime.datetime.now()
    solver.find_shortest_path()
    print 'find path end', datetime.datetime.now()
    world = World(size, obstacle_creator.map, solver.distance, solver.parent, solver.shortest_path)
    world.world_reset()
    print 'world reset', datetime.datetime.now()
    world.draw_distance()
    print 'draw_distance end', datetime.datetime.now()
    world.draw_shortest_path()
    print 'shortest_path', datetime.datetime.now()
    #world.draw_parent()
    cv2.imshow('image', world.img), cv2.waitKey(1)
cv2.destroyAllWindows()
