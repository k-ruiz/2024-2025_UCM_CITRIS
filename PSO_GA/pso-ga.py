import numpy as np
import matplotlib.pyplot as plt
from pyswarm import pso  #for PSO 
from deap import base, creator, tools, algorithms  #for GA

lat_min, lat_max = 36.5, 38.5  #(best guesstimate) of bounds from UC Merced to UC Davis 
lon_min, lon_max = -123.0, -121.5
alt_min, alt_max = 300, 1000  #meters above sea level 


def fitness_function(coords, optimizer_type = 'PSO'):
    x, y, z = coords
    target_coords = [37.8715, -122.2730, 500]  # cords for berkeley
    distance = np.sqrt((x - target_coords[0])**2 + (y - target_coords[1])**2 + (z - target_coords[2])**2)
    avg_speed = 75  # m/s
    flight_time = distance / avg_speed
    energy_usage = 0.05 * distance + 0.1 * 10  # Arbitrary energy formula
    
    # Weights for time and energy
    w_time = 0.6
    w_energy = 0.4
    fitness_value = w_time * flight_time + w_energy * energy_usage

    if optimizer_type == "PSO":
        return fitness_value #keeps this as a float/scalar for PSO 
    elif optimizer_type == "GA":
        return (fitness_value,) #i hope this changes it to a tuple 
    else:
        raise ValueError("Unknown Optimizer Type")

def pso_optimization():
    lb = [lat_min, lon_min, alt_min]  #lower bound for dimension
    ub = [lat_max, lon_max, alt_max]    #upper bound 
    
    #actual run of PSO 
    best_solution, best_value = pso(lambda coords: fitness_function(coords, optimizer_type = 'PSO'), lb, ub, swarmsize = 50, maxiter = 100)
    print(f"Best Solution (PSO): {best_solution}")
    print(f"Best Value (PSO): {best_value}")

#37.8712N -122.235W these are the coords eduardo gave me, but i just did like a typo thing on the actual bounds so  



def ga_optimization():
    creator.create("fitnessmin", base.Fitness, weights = (1.0,))
    creator.create("individual", list, fitness = creator.fitnessmin)
    
    toolbox = base.Toolbox()
    toolbox.register("attr_lat", np.random.uniform, lat_min, lat_max)
    toolbox.register("attr_lon", np.random.uniform, lon_min, lon_max)
    toolbox.register("attr_alt", np.random.uniform, alt_min, alt_max)

    def create_individual():
        return creator.individual([toolbox.attr_lat(), toolbox.attr_lon(), toolbox.attr_alt()])
    
    toolbox.register("individual", create_individual)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    toolbox.register("mate", tools.cxBlend, alpha = 0.5)
    toolbox.register("mutate", tools.mutGaussian, mu = 0, sigma = 1, indpb = 0.2)
    toolbox.register("select", tools.selTournament, tournsize = 3)
    toolbox.register("evaluate", lambda individual: fitness_function(individual, optimizer_type = 'GA'))

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

