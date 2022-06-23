#genetic algorithm start
import numpy as np
import sim_tools.sim as sim
import sim_tools.media_export as export
import imitation_tools.automation as automation
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

if __name__ ==  '__main__':
    #constants to imitate
    k_coh = 3
    k_align = 5
    k_sep = .5
    k_inertia = 1

    true_gains = [k_coh, k_align, k_sep, k_inertia]

    #run sim
    print("Generating Initial")
    controllers = [bo.Boids(*true_gains) for i in range(params.num_agents)]
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

    if not os.path.exists("GeneticOutput"):
        os.makedirs("GeneticOutput")

    export.export(export.ExportType.GIF,"GeneticOutput/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)


    posVelSlices = []
    #run tons more short sims
    shortSimParams = copy.deepcopy(params)
    print("Running short sims")
    shortSimParams.overall_time = 5


    agentSlices = automation.runSims(controllers,params=shortSimParams,num_sims=1)

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
    num_generations = 30
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

    # now create some hybrid visualizations
    print("Running hybrid visual")
    #some parameters for hybrid visualization
    mix_factor = 0.6
    params.num_agents = 100
    params.enclosure_size = 20
    params.overall_time = 20
    params.init_pos_max = params.enclosure_size
    params.agent_max_vel = 7

    original_agents = [bo.Boids(*true_gains) for i in range(int(params.num_agents*mix_factor))]
    for controller in original_agents:
        controller.setColor("rgb(99, 110, 250)")
    imitated_agents = [bo.Boids(solution[0],solution[1],solution[2],k_inertia) for i in range(int(params.num_agents*(1-mix_factor)))]
    for controller in imitated_agents:
        controller.setColor("black")

    all_controllers = original_agents + imitated_agents

    agentPositions_hybrid, agentVels_hybrid = sim.runSim(all_controllers,params,progress_bar=True)
    export.export(export.ExportType.MP4,"GeneticOutput/Hybrid",agentPositions_hybrid,agentVels_hybrid,controllers=all_controllers,params=params,vision_mode=False,progress_bar=True)

    ga_instance.plot_fitness()