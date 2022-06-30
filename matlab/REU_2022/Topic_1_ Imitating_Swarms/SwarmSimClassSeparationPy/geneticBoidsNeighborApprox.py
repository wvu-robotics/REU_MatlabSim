from sklearn.model_selection import train_test_split
from sim_tools import sim
from sim_tools import media_export as export
from imitation_tools import data_prep
import numpy as np
import copy
import os
import pygad
from tqdm import tqdm
from sklearn.linear_model import LinearRegression as lr

from models.Boids import Boids

params = sim.SimParams(
    num_agents=40, 
    dt=0.01, 
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

if __name__ ==  '__main__':
    k_coh = 3
    k_align = 1
    k_sep = .2

    orig_controller = Boids(k_coh,k_align,k_sep,1)
    # really should clean up this interface too
    controllers = [copy.deepcopy(orig_controller) for i in range(params.num_agents)]
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

    if not os.path.exists("GeneticOutputNeighbor"):
        os.makedirs("GeneticOutputNeighbor")

    export.export(export.ExportType.MP4,"GeneticOutputNeighbor/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)

    shortSimParams = copy.deepcopy(params)
    print("Running short sim")
    shortSimParams.overall_time = 1
    num_sims = 4

    posVelSlices = []
    for i in range(num_sims):
        trainPositions, trainVels = sim.runSim(controllers,shortSimParams,progress_bar=True)
        posVelSlices.extend(data_prep.toPosVelSlices(trainPositions))

    def fitnessMicroExpanded(posVelSlices):
        def fitness(est_gains,sol_id):
            localParams = copy.deepcopy(shortSimParams)
            localParams.neighbor_radius = est_gains[3] #trying to learn
            est_gains = np.array(est_gains[:3]) #note a mutation, probably bad practice

            agentSlices = data_prep.toAgentSlices(posVelSlices,localParams,ignoreConstrainedMotion=False,verbose=False)
            # maybe subsample to speed up fitness calc
            maxSlices = 10000
            if len(agentSlices)>maxSlices:
                np.random.shuffle(agentSlices)
                agentSlices = agentSlices[:maxSlices]

            #print("Gains",est_gains)
            loss = 0.0
            for slice in agentSlices:
                vel_pred = np.dot(est_gains.transpose(),np.array([slice.cohesion,slice.alignment,slice.separation]))
                vel_pred = sim.motionConstraints(vel_pred,slice.last_vel,params)#, learn without constraints first...idk why this changes it
                err = vel_pred - slice.output_vel
                loss += np.linalg.norm(err) # could change to MSE, but I like accounting for direction better
            if len(agentSlices) == 0:
                return -np.inf
            return -loss/len(agentSlices) #normalized between runs
        return fitness

# is terrible
    def fitnessLinearReg(posVelSlices):
        def fitness(radius,sol_id):
            # only do genetic with on the neighbor radius, with fitness doing a linear regression
            localParams = copy.deepcopy(shortSimParams)
            localParams.neighbor_radius = radius[0]

            # hrm maybe should be constraining motion more
            agentSlices = data_prep.toAgentSlices(posVelSlices,localParams,ignoreConstrainedMotion=False,verbose=False)
            # maybe subsample to speed up fitness calc
            maxSlices = 1000
            if len(agentSlices)>maxSlices:
                np.random.shuffle(agentSlices)
                agentSlices = agentSlices[:maxSlices]
            
            if len(agentSlices) == 0:
                return -np.inf
            
            # estimate boids gains
            #prep for linear regression
            X = []
            y = []
            for slice in agentSlices:
                X.append([slice.cohesion[0],slice.alignment[0],slice.separation[0]])
                y.append(slice.output_vel[0])
                X.append([slice.cohesion[1],slice.alignment[1],slice.separation[1]])
                y.append(slice.output_vel[1])
            X = np.array(X)
            y = np.array(y)
            # fit linear regression
            reg = lr()
            reg.fit(X,y)

            return reg.score(X,y)
            est_gains = reg.coef_

            # loss = 0.0
            # for slice in agentSlices:
            #     vel_pred = np.dot(est_gains,np.array([slice.cohesion,slice.alignment,slice.separation]))
            #     vel_pred = sim.motionConstraints(vel_pred,slice.last_vel,params)#, learn without constraints first...idk why this changes it
            #     err = vel_pred - slice.output_vel
            #     loss += np.linalg.norm(err) # could change to MSE, but I like accounting for direction better
            # if len(agentSlices) == 0:
            #     return -np.inf
            return -loss/len(agentSlices) #normalized between runs
        return fitness
        


    fitnessFunc = fitnessMicroExpanded(posVelSlices)
    #this is higher than I would like
    print("True Loss:",fitnessFunc([k_coh,k_align,k_sep,shortSimParams.neighbor_radius],0))
    
    print("Running genetic algo:")
    num_generations=15
    with tqdm(total=num_generations) as pbar:
        ga_instance = pygad.GA(
            num_generations=num_generations,
            num_parents_mating=6,
            fitness_func=fitnessFunc,
            sol_per_pop=15,
            num_genes=4,
            # mutation_type="adaptive",
            mutation_probability=1,
            gene_type=float,
            gene_space = [{'low': 0, 'high': 10},{'low': 0, 'high': 10}, {'low': 0, 'high': 10},{'low': 0, 'high': 10}],
            on_generation=lambda _:pbar.update(1) #make progress bar work
        )
        ga_instance.run()
        solution, solution_fitness, solution_idx = ga_instance.best_solution(ga_instance.last_generation_fitness)
        print("Parameters of the best solution : {solution}".format(solution=solution))
        print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))

        print("Running final sim:")
        imitation = Boids(solution[0],solution[1],solution[2])
        params.neighbor_radius = solution[3]
        imitated_controllers = [copy.deepcopy(imitation) for i in range(params.num_agents)]
        for controller in imitated_controllers:
            controller.setColor("red")
        imitated_agentPositions, imitated_agentVels = sim.runSim(imitated_controllers,params,progress_bar=True,initial_positions=agentPositions[0],initial_velocities=agentVels[0])
        export.export(export.ExportType.MP4,"GeneticOutputNeighbor/Imitation",imitated_agentPositions,imitated_agentVels,params=params,vision_mode=False,progress_bar=True,controllers=imitated_controllers)


        ga_instance.plot_fitness()
