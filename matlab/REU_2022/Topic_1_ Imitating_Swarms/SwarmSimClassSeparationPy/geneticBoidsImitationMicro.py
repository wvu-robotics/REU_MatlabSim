#genetic algorithm start
import numpy as np
import sim_tools.sim as sim
import sim_tools.media_export as export
from tqdm import tqdm
import copy
from dataclasses import dataclass #data class is like the python equivalent of a struct
from timeit import default_timer as timer
import os

import pygad

from models import Boids as bo
from models import BoidsNonLinear as nl

params = sim.SimParams(
    num_agents=40,
    dt=0.1,
    overall_time = 15,
    enclosure_size = 10,
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=5,
    init_vel_max = None,
    agent_max_accel=2,
    agent_max_turn_rate=4*np.pi,
    neighbor_radius=3,
    periodic_boundary=False
    )

#constants to imitate
k_coh = 3
k_align = 5
k_sep = .1
k_inertia = 1

true_gains = [k_coh, k_align, k_sep, k_inertia]

#run sim
print("Generating Initial")
controllers = [bo.Boids(*true_gains) for i in range(params.num_agents)]
agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

if not os.path.exists("GeneticOutput"):
    os.makedirs("GeneticOutput")

export.export(export.ExportType.GIF,"GeneticOutput/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)

# print("Agent positions:")
# print(agentPositions)
# print("Agent velocities:")
# print(agentVels)

#reformat positions and vels by slice
@dataclass
class posVelSlice:
    pos: np.ndarray
    vel: np.ndarray
    next_vel: np.ndarray

posVelSlices = [posVelSlice(agentPositions[i],agentVels[i],agentVels[i+1]) for i in range(len(agentPositions)-1)]

# print("PosVelSlices:")
# print(posVelSlices)

posVelSlices = []
#run tons more short sims
shortSimParams = copy.deepcopy(params)
print("Running short sims")
shortSimParams.overall_time = 5
# shortSimParams.enclosure_size = 2*params.enclosure_size
# shortSimParams.init_pos_max = params.enclosure_size/2

extra_sims = 1
for extra_sim in tqdm(range(extra_sims)):
    agentPositions, agentVels = sim.runSim(controllers,shortSimParams)
    posVelSlices.extend(
    [posVelSlice(agentPositions[i],(
        agentPositions[i]-agentPositions[i-1])/params.dt,
        (agentPositions[i+1]-agentPositions[i])/params.dt) 
    for i in range(1,len(agentPositions)-1)])

#reformat slices to be relevant by agent, getting the centroid, separation, alignment, and last vel
@dataclass
class agentSlice:
    #inputs
    cohesion: np.ndarray
    alignment: np.ndarray
    separation: np.ndarray
    last_vel: np.ndarray
    #output
    output_vel: np.ndarray

agentSlices = []
for slice in posVelSlices:
    for agent in range(params.num_agents):
        #calculate all relevant derivate metrics, repackage
        posCentroid = np.zeros(2)
        velCentroid = np.zeros(2)
        agentPos = slice.pos[agent]
    
        #throw out data near the boundary, only needed with bounce
        if(agentPos[0] > params.enclosure_size or agentPos[0] < -params.enclosure_size 
        or agentPos[1] > params.enclosure_size or agentPos[1] < -params.enclosure_size):
            continue

        agentVel = slice.vel[agent]
        agentNextVel = slice.next_vel[agent]
        separation = np.zeros(2)

        adjacent = 0
        for otherAgent in range(params.num_agents):
            if otherAgent == agent:
                continue
            
            otherPos = slice.pos[otherAgent]

            if np.linalg.norm(otherPos-agentPos) > params.neighbor_radius:
                continue
            
            dist = np.linalg.norm(otherPos-agentPos)

            separation += ((otherPos-agentPos)/dist)*-1*(1/(dist**6))
            otherVel = slice.vel[otherAgent]
            posCentroid += otherPos
            velCentroid += otherVel
            adjacent += 1
        
        #throw out data without interactions
        if adjacent < 1:
            continue

        posCentroid /= adjacent
        velCentroid /= adjacent

        agentVelDifference = agentNextVel

        agentSlices.append(agentSlice(posCentroid-agentPos,velCentroid,separation,agentVel,agentVelDifference))

#sanity check, this should have 0 loss, anything else is either noise or bad measurements
#this is a bit high and I don't like it
true_loss = 0
for slice in agentSlices:
    gains_ex = true_gains[:3]
    args = np.array([slice.cohesion,slice.alignment,slice.separation])
    # print("Args",args)
    # print("Gains",gains_ex)
    vel_pred = np.dot(args.transpose(),gains_ex)
    
    vel_pred = sim.motionConstraints(vel_pred,slice.last_vel,params)

    err = vel_pred - slice.output_vel
    #print("err",err)
    true_loss += np.linalg.norm(err)
true_loss/=len(agentSlices)
print("True loss(data deviation from original on average):",true_loss)

print("Ignored ",(len(posVelSlices)*params.num_agents)-len(agentSlices),"/",len(posVelSlices)*params.num_agents," slices")

#using function currying to make this a tad more useful
def sliceBasedFitness(agentSlices=[]):
    def fitness(est_gains,solution_idx):
        loss = 0.0
        for slice in agentSlices:
            vel_pred = np.dot(est_gains.transpose(),np.array([slice.cohesion,slice.alignment,slice.separation]))
            vel_pred = sim.motionConstraints(vel_pred,slice.last_vel,params)
            err = vel_pred - slice.output_vel
            loss += np.linalg.norm(err) # could change to MSE, but I like accounting for direction better
        return -loss/len(agentSlices) #normalized between runs
    return fitness

fitness_function = sliceBasedFitness(agentSlices)
num_generations = 50
num_parents_mating = 4

sol_per_pop = 8
num_genes = 3

#bounds for gains, will narrow if it's annoying
init_range_low = 0
init_range_high = 5

parent_selection_type = "sss"
keep_parents = 1

crossover_type = "single_point"

mutation_type = "random"
mutation_percent_genes = 100

ga_instance = pygad.GA(num_generations=num_generations,
                       num_parents_mating=num_parents_mating,
                       fitness_func=fitness_function,
                       sol_per_pop=sol_per_pop,
                       num_genes=num_genes,
                       init_range_low=init_range_low,
                       init_range_high=init_range_high,
                       parent_selection_type=parent_selection_type,
                       keep_parents=keep_parents,
                       crossover_type=crossover_type,
                       mutation_type=mutation_type,
                       mutation_percent_genes=mutation_percent_genes)

start = timer()
ga_instance.run()
print("Took ",timer()-start," seconds")

solution, solution_fitness, solution_idx = ga_instance.best_solution()
print("Parameters of the best solution : {solution}".format(solution=solution))
print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))

controllers_imitated = [bo.Boids(solution[0],solution[1],solution[2],k_inertia) for i in range(params.num_agents)]
for controller in controllers_imitated:
    controller.setColor('black')
agentPositions_imitated, agentVels_imitated = sim.runSim(controllers_imitated,params,progress_bar=True)
export.export(export.ExportType.GIF,"GeneticOutput/Imitated",agentPositions_imitated,agentVels_imitated,controllers=controllers_imitated,params=params,progress_bar=True)

ga_instance.plot_fitness()