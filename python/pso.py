"""\
Particle Swarm Optimization

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import numpy
from random import uniform


class Particle(numpy.ndarray):
    _gbest = None
    _gbest_fitness = None

    def __init__(self, *args):
        self.velocity = numpy.ndarray(self.shape[0])
        self._best = None
        self._best_fitness = None

    @property
    def best(self):
        return (self._best, self._best_fitness)

    @property
    def gbest(self):
        return (self.__class__._gbest, self.__class__._gbest_fitness)

    def update(self, omega, phip, phig):
        for d in range(self.shape[0]):
            rp, rg = uniform(0, 1), uniform(0, 1)
            self.velocity[d] = omega * self.velocity[d] \
                + phip * rp * (self._best[d] - self[d]) \
                + phig * rg * (self._gbest[d] - self[d])
        self += self.velocity

    def update_best(self, fitness):
        if fitness > self._best_fitness:
            self._best = tuple(self)
            self._best_fitness = fitness
            if fitness > self.__class__._gbest_fitness:
                self.__class__._gbest = tuple(self)
                self.__class__._gbest_fitness = fitness


def particle_swarm_optimize(fitness, dim, bounds, size, omega, phip, phig, it, af):
    particles = [Particle(dim) for i in range(size)]
    for particle in particles:
        for d in range(dim):
            particle[d] = uniform(bounds[d][0], bounds[d][1])
            span = bounds[d][1] - bounds[d][0]
            particle.velocity[d] = uniform(-span, span)
    for i in range(it):
        for particle in particles:
            particle.update_best(fitness(particle))
        for particle in particles:
            particle.update(omega, phip, phig)
        if not particles[0].gbest[1] < af:
            break
    return particles[0].gbest
