#!/usr/bin/env python

'''
A simple implementation of DPSO.
Author: Daniel Karlsson, Royal Institute of Technology, Sweden, danieka@kth.se 
Date: 2013-11-27
Dependency: matplotlib

The basic principle is that the problem is considered as optimising the position of a number of 
waypoints so that the distance traveled is minimized. If no valid route can be found with the 
current number of waypoints a new waypoint is added.

Algoritm:
1. Start of with a set number of waypoints, for example 3.
2. Generate a population of particles where the waypoints are randomly placed, 
    one path should be a straight line from start to goal.
3. For each particle do {
    3.1 Calculate fitness for each particle, replace gbest and pbest accordingly
    3.2 Update speed
    3.3 Update position
    3.4 Run until end condition has been met, optimal route found or time limit exceeded or no more improvement.
4. If the best route intersects with an obstacle: Add a waypoint and goto 2.
   If the best route does not intersect an obstacle: Finish
'''


from random import uniform,random, randint
from math import sin, sqrt
import matplotlib.pyplot as plt
from matplotlib.path import Path
from copy import deepcopy


pop_size = 1000                                         #The number of particles in the swarm
waypoints = 3                                           #The number of waypoints the path is divided in
iter_max = 200                                          #The maximum number of iterations the simulation should run
c1 = 4                                                  #c1 constant, used when calculating particle speed, see paper for details
c2 = 4                                                  #c2 constant, used when calculating particle speed, see paper for details
goal = [10,10]                                          #The co-ordinates of the goal
inertia = 0.7                                           #inertia constant, w, used when calculating particle speed, see paper for details
total_distance = sqrt(pow(goal[0],2) + pow(goal[1],2))  #The euclidian distance between starting point and goal, used when calculating fitness
n=5                                                     #The size of the neighborhood, see paper for more details

#Set up plot
plot = [[0],[0]]
fig = plt.figure()
plt.axis([0, goal[0], 0, goal[1]])
plt.show(block=False)
codes = [Path.MOVETO,
         Path.LINETO,
         ]

#Randomly generate a couple of obstacles
obstacles = []
for i in range(20):
    rectangle = plt.Rectangle((randint(1,goal[0]-2),randint(1,goal[1]-2)), 1,1)
    obstacles.append(rectangle)
    plt.gca().add_patch(rectangle)



def draw_obstacles():
    """This function draws the obstacles and must be called every time the plot is updated"""
    for obstacle in obstacles:
        plt.gca().add_patch(obstacle)

class Particle:
    """Placeholder-class for all the particles"""
    pass
 
def evaluate(inp, coll = False):
    """This evaluates the fitness of each route calculating the distance travelled and adds a penalty for intersecting objects"""
    distance = 0
    errors = 0
    waypoints = [[0,0]]                 #The robot originates in 0,0
    waypoints.extend(inp)               #Add the generated waypoints inbetween start and goal
    waypoints.append([goal[0],goal[1]]) #Add the goal waypoint to complete the route
    for i in range(0, len(waypoints) -1):
        distance += sqrt(pow(waypoints[i][0] - waypoints[i+1][0], 2) + pow(waypoints[i][1] - waypoints[i+1][1], 2)) #Calculates the distance between this waypoint ond the next
       
        errors += collisions(waypoints[i][0], waypoints[i][1],            
              waypoints[i+1][0], waypoints[i+1][1])

    if collisions:
        return distance + errors*total_distance*2, errors
    return distance + errors*total_distance*2

def collisions(x1,y1,x2,y2):
    """Calculates the number of collisions"""
    collisions = 0
    m = (y2 - y1) / (x2 - x1)
    y = lambda x: m*(x-x1) + y1
    
    under = None
    for obstacle in obstacles:
        rx = obstacle.get_x()
        ry = obstacle.get_y()
        rh = obstacle.get_height()
        rw = obstacle.get_width()
        
        intersects = False   
        if y(x) < ry + rh and y(rx) > ry:
            intersects = True
        if y(rx) > ry + rh and y(rx+rw) < ry+rh:
            intersects = True
        if y(rx) < ry + rh and y(rx+rw) > ry+rh:
            intersects = True
        if y(rx) > ry and y(rx+rw) < ry:
            intersects = True
        if y(rx) < ry and y(rx+rw) > ry:
            intersects = True    
        
        if intersects:
            collisions += 1
                    
    return collisions

#initialize the particles
particles = []
#This is the interval that should be between waypoints, see the paper for the whole algoritm which generates waypoints
kint = (goal[1]*2) / (waypoints + 1)
for i in range(pop_size):
    p = Particle()
    p.params = []
    for k in range(1,waypoints+1):
        if kint*k > goal[1]:
            x = uniform((kint*k - goal[0]) ,goal[0])
        else:
            x = uniform(0, kint*k)
        p.params.append([x, (kint*k - x)])
        
    p.fitness = evaluate(p.params)
    p.v = []
    for i in range(waypoints):
        p.v.append([random(), random()]) #Initialize with random speeds
    particles.append(p)
    
# let the first particle be the global best
gbest = particles[0]
for p in particles:
    p.best = p.params #set particles best to initial state

i = 0 #the number of iterations
ibest = 0 #the number of the iteration with the best result hitherto 
while i < iter_max and i - ibest < 10: #End if no iprovement in 10 iterations
    for p in particles:
        #Calculate fitness
        fitness = evaluate(p.params)
        if fitness < p.fitness:         #If new best route constructed
            p.fitness = fitness
            p.best = deepcopy(p.params)

            if fitness < gbest.fitness: #If the new best is also the global best
                gbest = deepcopy(p)
                ibest = i
                
        
    for j, p in enumerate(particles):
        #Calculate speed
        nbest = p
        for l in range(j - ((n/2%1)), j + ((n/2%1))):
            if particles[l%len(particles)].fitness < nbest.fitness:
                nbest = particles[l]
        
        for l, point in enumerate(p.params):
            #Here we update the speeds according to canonical PSO, see paper for detail
            p.v[l][0] = inertia*p.v[l][0] + c1*random()*(p.best[l][0] - point[0]) + c2*random()*(nbest.params[l][0] - point[0])     
            p.v[l][0] = inertia*p.v[l][1] + c1*random()*(p.best[l][1] - point[1]) + c2*random()*(nbest.params[l][1] - point[1])
     
    for particle in particles:
        for l, point in enumerate(particle.params):
            #update position
            point[0] += p.v[l][0]
            point[1] += p.v[l][1]
        
    i  += 1

    plt.cla()                   #clear plot
    plot = [[0],[0]]
    for point in gbest.params:
        plot[0].append(point[0])
        plot[1].append(point[1])
    plot[0].append(goal[0])
    plot[1].append(goal[1])

    plt.plot(plot[0], plot[1], "r")
    plt.axis([0, goal[0], 0, goal[1]])
    draw_obstacles()
    plt.draw()


print "Best", evaluate(gbest.params, coll = True)
plt.show()
