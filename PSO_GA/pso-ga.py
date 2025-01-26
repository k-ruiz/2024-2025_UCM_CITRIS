import numpy as np
import matplotlib.pyplot as plt
from pyswarm import pso  # For PSO
from deap import base, creator, tools, algorithms  # For GA

def fitness_function(x):
    #uhh i have no idea what to put here rn but yeah fitness function goes here 
    return (sum([xi**2 for xi in x]),)

def pso_optimization():
    lb = [-10, -10]  #lower bound for dimension
    ub = [10, 10]    #upper bound 
    
    #actual run of PSO 
    best_solution, best_value = pso(fitness_function, lb, ub, swarmsize = 50, maxiter = 100)
    print(f"Best Solution (PSO): {best_solution}")
    print(f"Best Value (PSO): {best_value}")

def ga_optimization():
    creator.create("fitnessmin", base.Fitness, weights = (1.0,))
    creator.create("individual", list, fitness = creator.fitnessmin)
    
    toolbox = base.Toolbox()
    toolbox.register("attr_float", np.random.uniform, -10, 10)
    toolbox.register("individual", tools.initRepeat, creator.individual, toolbox.attr_float, n=2)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    toolbox.register("mate", tools.cxBlend, alpha = 0.5)
    toolbox.register("mutate", tools.mutGaussian, mu = 0, sigma = 1, indpb = 0.2)
    toolbox.register("select", tools.selTournament, tournsize = 3)
    toolbox.register("evaluate", fitness_function)

    population = toolbox.population(n = 50)
    ngen = 100 
    cxpb, mutpb = 0.7, 0.2 #crossover and mutation probabilities

    result, log = algorithms.eaSimple(population, toolbox, cxpb, mutpb, ngen, stats = None, verbose = False)
    best_individual = tools.selBest(population, k = 1)[0]
    print(f"Best Solution (GA): {best_individual}")
    print(f"Best value (GA): {fitness_function(best_individual)}")
 
#is this where i put the main script idk if the function calls are good here 

if __name__ == "__main__":
    print("Running PSO Optimization:")
    pso_optimization()
    
    print("\nRunning GA Optimization:")
    ga_optimization()

