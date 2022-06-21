from random import random
from matplotlib.pyplot import get
import numpy as np
from sim_tools import sim
from tqdm import tqdm
from dataclasses import dataclass #data class is like the python equivalent of a struct
from models import Boids as bo
from macro_metrics_extract import get_metrics
import pygad 
from sim_tools import media_export as export


params = sim.SimParams(
    num_agents=20, 
    dt=0.05, 
    overall_time = 30, 
    enclosure_size = 10, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=7,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=2,
    periodic_boundary=False
    )


#constants to imitate
k_coh = 3
k_align = 3
k_sep = 4
k_inertia = 1

true_gains = [k_coh, k_align, k_sep, k_inertia]

#run sim
# print("Original agent slices")


controllers = [bo.Boids(*true_gains) for i in range(params.num_agents)]
print("First sim and export")
agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

for controller in controllers:
       controller.setColor("blue")
media_type = export.ExportType.MP4

export.export(media_type,"initial",agentPositions,agentVels,controllers=controllers,params=params,vision_mode=False,progress_bar=True)
print("Media generated")


target_metrics=get_metrics(agentPositions,agentVels,params)
desired_output = target_metrics

def fitness_func(solution, solution_idx):
    imitationgains=[*solution,k_inertia]
        
    controllersimitation = [bo.Boids(*imitationgains) for i in range(params.num_agents)]
    agentPositionsI, agentVelsI = sim.runSim(controllersimitation,params,progress_bar=True)
    metrics=get_metrics(agentPositionsI,agentVelsI,params)
    # print(metrics)
    fitness1 = 1.0 / (np.abs(metrics[0] - desired_output[0]) + 0.000001)
    fitness2 = 1.0 / (np.abs(metrics[1] - desired_output[1]) + 0.000001)
    fitness3 = 1.0 / (np.abs(metrics[2] - desired_output[2]) + 0.000001)
    fitness= 0.3*fitness1 + 0.3*fitness2 + 0.4*fitness3
    return fitness

fitness_function=fitness_func
def on_start(ga_instance):
    print("on_start()")

def on_fitness(ga_instance, population_fitness):
    print("on_fitness()")

def on_parents(ga_instance, selected_parents):
    print("on_parents()")

def on_crossover(ga_instance, offspring_crossover):
    print("on_crossover()")

def on_mutation(ga_instance, offspring_mutation):
    print("on_mutation()")

def on_generation(ga_instance):
    print("on_generation()")

def on_stop(ga_instance, last_population_fitness):
    print("on_stop()")

ga_instance = pygad.GA(num_generations=15,
                       num_parents_mating=3,
                       fitness_func=fitness_function,
                       sol_per_pop=8,
                       num_genes=3,
                       gene_type=float,
                       gene_space = [{'low': 0, 'high': 10}, {'low': 0, 'high': 10}, {'low': 0, 'high': 10}],
                       on_start=on_start,
                       on_fitness=on_fitness,
                       on_parents=on_parents,
                       on_crossover=on_crossover,
                       on_mutation=on_mutation,
                       on_generation=on_generation,
                       on_stop=on_stop)
print("Initial Population")
print(ga_instance.initial_population)

ga_instance.run()


print("Final Population")
print(ga_instance.population)

print("target metrics")
print(target_metrics)

ga_instance.plot_fitness()
solution, solution_fitness, solution_idx = ga_instance.best_solution(ga_instance.last_generation_fitness)
print("Parameters of the best solution : {solution}".format(solution=solution))
print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
print("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))

imitated_gains=solution
controllers = [bo.Boids(*imitated_gains,k_inertia) for i in range(params.num_agents)]
for controller in controllers:
       controller.setColor("black")
agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)
export.export(media_type,"Imitated",agentPositions,agentVels,controllers=controllers,params=params,vision_mode=False,progress_bar=True)