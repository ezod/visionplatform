"""\
Particle Swarm Optimization

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import numpy
from operator import itemgetter
from random import uniform


class Particle(numpy.ndarray):
    _gbest = None
    _gbest_fitness = None
    _gbest_fitness_e = None

    def __init__(self, *args):
        self.velocity = numpy.ndarray(self.shape[0])
        self._best = None
        self._best_fitness = None
        self._best_fitness_e = None
        self.neighborhood = set()

    @property
    def best(self):
        return (self._best, self._best_fitness, self._best_fitness_e)

    @property
    def nbest(self):
        if not self.neighborhood:
            return self.gbest
        candidates = [particle.best for particle in self.neighborhood]
        return max(candidates, key=itemgetter(1,2))

    @property
    def gbest(self):
        return (self.__class__._gbest, self.__class__._gbest_fitness, self.__class__._gbest_fitness_e)

    def update(self, omega, phip, phig, constraint, bounds):
        for d in range(self.shape[0]):
            rp, rg = uniform(0, 1), uniform(0, 1)
            self.velocity[d] = omega * self.velocity[d] \
                + phip * rp * (self.best[0][d] - self[d]) \
                + phig * rg * (self.nbest[0][d] - self[d])
        self += self.velocity
        constraint(self, bounds)

    def update_best(self, fitness, fitness_e):
        F = fitness(self)
        if F > self._best_fitness:
            self._best = tuple(self)
            self._best_fitness = F
            self._best_fitness_e = -float('inf')
            if F > self.__class__._gbest_fitness:
                self.__class__._gbest = tuple(self)
                self.__class__._gbest_fitness = F
                self.__class__._gbest_fitness_e = -float('inf')
        if F == self.__class__._gbest_fitness:
            Fe = fitness_e(self)
            if Fe > self._best_fitness_e:
                self._best = tuple(self)
                self._best_fitness = F
                self._best_fitness_e = Fe
            if Fe > self.__class__._gbest_fitness_e:
                self.__class__._gbest = tuple(self)
                self.__class__._gbest_fitness = F
                self.__class__._gbest_fitness_e = Fe


topologies = {None: lambda particles: None}

def topology(f):
    topologies[f.__name__] = f
    return f

@topology
def ring(particles):
    for i, particle in enumerate(particles):
        particle.neighborhood.add(particle)
        particle.neighborhood.add(particles[i - 1])
        particle.neighborhood.add(particles[(i + 1) % len(particles)])

@topology
def star(particles):
    for particle in particles[1:]:
        particle.neighborhood.add(particle)
        particle.neighborhood.add(particles[0])


constraints = {None: lambda particle, bounds: None}

def constraint(f):
    constraints[f.__name__] = f
    return f

@constraint
def nearest(particle, bounds):
    for d in range(particle.shape[0]):
        particle[d] = max(particle[d], bounds[d][0])
        particle[d] = min(particle[d], bounds[d][1])

@constraint
def random(particle, bounds):
    for d in range(particle.shape[0]):
        if particle[d] < bounds[d][0] or particle[d] > bounds[d][1]:
            particle[d] = uniform(bounds[d][0], bounds[d][1])


def particle_swarm_optimize(fitness, fitness_e, dimension, bounds, size, omega,
                            phip, phig, it=None, af=float('inf'),
                            topology_type=None, constraint_type=None):
    particles = [Particle(dimension) for i in range(size)]
    for particle in particles:
        for d in range(dimension):
            particle[d] = uniform(bounds[d][0], bounds[d][1])
            span = bounds[d][1] - bounds[d][0]
            particle.velocity[d] = uniform(-span, span)
    topologies[topology_type](particles)
    i = 0
    while not it or i < it:
        for particle in particles:
            particle.update_best(fitness, fitness_e)
        for particle in particles:
            particle.update(omega, phip, phig, constraints[constraint_type], bounds)
        yield particles[0].gbest
        if particles[0].gbest[1] >= af and \
            (particles[0].gbest[2] == -float('inf') or particles[0].gbest[2] >= af):
            break
        i += 1
