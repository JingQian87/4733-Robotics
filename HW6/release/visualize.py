# usage:  python visualize.py world_file num_particles num_iterations

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import argparse
from copy import deepcopy

from robot import *

def read_txt(filename):
    file = open(filename, 'r')
    int_list = []
    for line in file:
        line = [int(x) for x in line.split()]
        int_list.append(line)
    world_size = int_list[0]

    landmarks = []
    for i in range(1, len(int_list)):
        landmarks.append(int_list[i])
    file.close()
    return (world_size, landmarks)


def draw_landmarks(landmarks, ax):
    landmark_size = 6
    for l in landmarks:
        ax.add_patch(patches.Rectangle((l[0]-landmark_size/2, l[1]-landmark_size/2), landmark_size, landmark_size, angle=0, color="orange"))

def draw_bot(robot, ax):
    ax.add_patch(patches.Circle((robot.x, robot.y), radius=2, color="red"))

def draw_particles(particles, color_number, ax):
    #length of color array = 11 shades of blue
    color_array = ["#e6ecff", "#b3c6ff", "#809fff", "#4d79ff", "#1a53ff", "#0039e6", "#002db3","#002080", "#00134d", "#000d33", "#000000"]
    for p in particles:
        ax.add_patch(patches.Circle((p.x, p.y), radius=1, color=color_array[color_number]))

def init_particles(world_size, number):
    particles = []
    for i in range(number):
        p = Robot(world_size)
        particles.append(p)
    return particles


def Gaussian_prob(mu, sigma, x):
    """ Credit to Prof. Peter Allen! :-) """
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

def measurement_prob(particle, robot_measurement):
    """ Credit to Prof. Peter Allen! :-) """
    # calculates how likely a measurement should be
    prob = 1.0
    
    particle_measurement = particle.sense()

    print("Particle at ({}, {})".format(particle.x, particle.y))

    # If in landmark or outside environ, assign 0
    if (not particle.x > 0) or (not particle.x < particle_measurement[0][0]):
        print("Particle ran away!")
        return 0.0
    if (not particle.y > 0) or (not particle.y < particle_measurement[0][1]):
        print("Particle ran away!")
        return 0.0
    
    half_diag = 1.42 * landmark_size * 0.5
    for i in range(1, len(particle_measurement)): # each is (dist, angle) pair
        if particle_measurement[i][0] < half_diag: # treat each obstacle as circle for simplicity
            return 0.0

    for i in range(1, len(particle_measurement)):
        prob *= Gaussian_prob(particle_measurement[i][0], particle.range_noise, robot_measurement[i][0])
        prob *= Gaussian_prob(particle_measurement[i][1], particle.bearing_noise, robot_measurement[i][1])
    
    return prob

# TODO: Move robot, make observation, and move and resample all particles
def update(robot, particles):
    """
    This function should first have the robot move and take a measurement. 
    Each particle should then move using the robot’s input vector uk and the move particle function 
    and then get a weight assigned to it.
    """
    # Move the robot
    uk = robot.move()
    # print(uk)

    # Take measurements
    Z = robot.sense()

    # Move the particles
    for p in particles:
        # print(p.x, p.y)
        p.move_particle(*uk)
        # print(p.x, p.y)
    
    # Assign weights to particles
    weights, total_weight = [], 0.0
    for p in particles:
        w = measurement_prob(p, Z)
        weights.append(w)
        total_weight += w
    
    # Normalize weights
    for w in weights:
        w /= total_weight
    
    # Resample!
    new_particles = []
    N = len(particles)
    for i in range(N):
        new_particles.append(deepcopy(np.random.choice(particles, p=weights)))

    return robot, particles


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('world', help="File for world size and landmarks")
    parser.add_argument('num_particles', help="Number of particles")
    parser.add_argument('num_iterations', help="Number of iterations")
    args = parser.parse_args()
    world_size, landmarks = read_txt(args.world)

    fig, ax = plt.subplots()
    ax.set_xlim([0,world_size[0]])
    ax.set_ylim([0,world_size[1]])
    draw_landmarks(landmarks, ax)
    ax.axis('equal')
    plt.ion()
    plt.show()

    robot = Robot(world_size)
    robot.set_noise(0.5, 0.1, 5.0, 1.0) # provided uncertainties
    particles = init_particles(world_size, int(args.num_particles))

    # Need to set particle noise too!!!
    for p in particles:
        p.set_noise(0.5, 0.1, 5.0, 1.0)

    for i in range(int(args.num_iterations)):
        draw_bot(robot, ax)
        draw_particles(particles, i, ax)
        plt.draw()
        plt.pause(0.1)
        robot, particles = update(robot, particles)