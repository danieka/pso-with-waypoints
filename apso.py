#!/usr/bin/env python

'''
pso.py
Author: Daniel Karlsson, Royal Institute of Technology, Sweden, danieka@kth.se 2013-11-27
A simple implementation of the Accelerated Particle Swarm Optimisation Algorithm as described in:
@article{
year={2012},
issn={1433-5298},
journal={Artificial Life and Robotics},
volume={17},
number={2},
doi={10.1007/s10015-012-0051-3},
title={A faster path planner using accelerated particle swarm optimization},
url={http://dx.doi.org/10.1007/s10015-012-0051-3},
publisher={Springer Japan},
keywords={Artificial intelligence; Global path planning; Local path planning; SLAM; Swarm robotics},
author={Mohamed, AbdullahZawawi and Lee, SangHeon and Hsu, HungYao and Nath, Namrata},
pages={233-240},
language={English}
}

Uses Numpy for matrix operations. 
Based on code by: Pradeep Gowda 2009-03-16
'''

from numpy import array
from random import uniform,random, randint
from math import sin, sqrt
import matplotlib.pyplot as plt
from matplotlib.path import Path


pop_size = 300
dimensions = 2
iter_max = 200
c1 = 2
c2 = 2
max_distance = 2


waypoints = [[1,1]]
plot = [[0],[0]]

fig = plt.figure()
plt.axis([0, 10, 0, 10])
plt.plot(plot[0], plot[1], "ro")

obstacles = []
for i in range(20):
    circle = plt.Rectangle((randint(1,8),randint(1,8)), 1,1)
    obstacles.append(circle)
    plt.gca().add_patch(circle)

plt.show(block=False)

codes = [Path.MOVETO,
         Path.LINETO,
         ]

class Particle:
    pass
 
def evaluate(param, old):
    #print waypoints[-1]
    #if sqrt(pow(param[0] - old[0], 2) + pow(param[1] - old[1], 2)) > max_distance:
    #    return 900
    
    verts = [(old[0], old[1]),
              (param[0], param[1]),]
    
    path = Path(verts, codes)
    
    for obstacle in obstacles:
        if sqrt(pow(param[0] - obstacle.get_xy()[0], 2) + pow(param[1] - obstacle.get_xy()[1], 2)) < 10:
            if path.intersects_bbox(obstacle.get_bbox()):
                return 900

        
    return sqrt(pow(param[0] - 10, 2) + pow(param[1] - 10, 2))


#initialize the particles
particles = []
for i in range(pop_size):
    p = Particle()
    p.params = array([uniform(0, 1),  uniform(0, 1)])
    p.old = array([0,0])
    p.fitness = 1000
    p.v = 0.0
    particles.append(p)

# let the first particle be the global best
gbest = particles[0]
i = 0

while gbest.fitness > 0.1 and i < iter_max:
    for p in particles:
        fitness = evaluate(p.params, p.old)
        if fitness < p.fitness:
            p.fitness = fitness
            p.best = p.params

        if fitness < gbest.fitness:
            gbest = p
        p.old = p.params
        xv = (1- 0.4)*p.params[0] + 0.4*gbest.params[0] + 0.1*(random() - 0.5)        

        
        yv = (1- 0.4)*p.params[1] + 0.4*gbest.params[1] + 0.1*(random() - 0.5)

        p.params[1] = yv
        p.params[0] = xv
        
    i  += 1

    waypoints.append([gbest.params[0], gbest.params[1]])
    plot[0].append(gbest.params[0])
    plot[1].append(gbest.params[1]) 
    plt.plot(plot[0], plot[1], "ro")
    plt.draw()


    print gbest.fitness, gbest.params

plt.show()