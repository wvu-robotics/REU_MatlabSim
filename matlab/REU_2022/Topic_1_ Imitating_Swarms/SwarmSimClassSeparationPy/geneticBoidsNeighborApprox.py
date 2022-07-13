from sklearn.covariance import log_likelihood
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
import statsmodels.api as sm
from features.features import *

from models.Boids import Boids
from models.featureCombo import FeatureCombo as fc

params = sim.SimParams(
    num_agents=30, 
    dt=0.05, 
    overall_time = 15, 
    enclosure_size = 15, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=7,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=2,
    periodic_boundary=False
    )

if __name__ ==  '__main__':
    orig_features = [
        Cohesion(),
        Alignment(),
        SeparationInv2()
    ]

    true_gains = np.array([1,1,1])

    orig_controller = fc(true_gains,orig_features)
    # really should clean up this interface too
    controllers = [copy.deepcopy(orig_controller) for i in range(params.num_agents)]
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

    if not os.path.exists("GeneticOutputNeighbor"):
        os.makedirs("GeneticOutputNeighbor")

    export.export(export.ExportType.GIF,"GeneticOutputNeighbor/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)

    shortSimParams = copy.deepcopy(params)
    print("Running short sim")
    shortSimParams.num_agents = 10
    shortSimParams.enclosure_size = 10 #strong effect on learning separation
    shortSimParams.overall_time = 4
    shortSimParams.init_pos_max = shortSimParams.enclosure_size
    shortSimParams.agent_max_vel = 7
    num_sims = 4

    posVelSlices = []
    for i in range(num_sims):
        trainPositions, trainVels = sim.runSim(controllers,shortSimParams,progress_bar=True)
        posVelSlices.extend(data_prep.toPosVelSlices(trainPositions,shortSimParams))

    def fitnessMicroExpanded(posVelSlices):
        def fitness(est_gains,sol_id):
            localParams = copy.deepcopy(shortSimParams)
            localParams.neighbor_radius = est_gains[3] #trying to learn
            est_gains = np.array(est_gains[:3]) #note a mutation, probably bad practice

            agentSlices = data_prep.toAgentSlices(posVelSlices,localParams,ignoreConstrainedMotion=False,verbose=False)
            # maybe subsample to speed up fitness calc
            # maxSlices = 10000
            # if len(agentSlices)>maxSlices:
            #     np.random.shuffle(agentSlices)
            #     agentSlices = agentSlices[:maxSlices]

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

            learning_features = {
                "coh:":Cohesion(),
                "align:":Alignment(),
                "sep:":SeparationInv2()
            }
            # hrm maybe should be constraining motion more
            # I might migrate to new interface for speed
            agentSlices = data_prep.toFeatureSlices(posVelSlices,learning_features,localParams,verbose=False)
            
            if len(agentSlices) == 0:
                return -np.inf
            
            # estimate boids gains
            #prep for linear regression
            X = []
            y = []
            for slice in agentSlices:
                if slice.motion_constrained or slice.boundary_constrained:
                    continue

                X.append(np.array(list(slice.features.values()))[:,0])
                y.append(slice.output_vel[0]-slice.last_vel[0])
                X.append(np.array(list(slice.features.values()))[:,1])
                y.append(slice.output_vel[1]-slice.last_vel[1])
            if len(X)==0:
                return -np.inf
            
            X = np.array(X)
            y = np.array(y)
            # fit linear regression
            # reg = lr()
            # reg.fit(X,y)
            model = lr().fit(X,y)
            # print("Data ",len(X))
            est_gains = model.coef_
            # print("Radius ",radius[0],"Gains",est_gains)

            # true_loss = 0.0
            loss = 0.0
            for i in range(len(X)):
                v_pred = np.dot(est_gains.transpose(),X[i])
                v_pred_true = np.dot(true_gains.transpose(),X[i])
                v_actual = y[i]
                loss += np.sum(np.abs(v_pred-v_actual))
                # true_loss += np.sum(np.abs(v_pred_true-v_actual))

            # print("True loss",true_loss/len(X))
            return -loss/len(agentSlices) #normalized between runs
        return fitness
        


    fitnessFunc = fitnessLinearReg(posVelSlices)
    #this is higher than I would like
    print("True Fitness:",fitnessFunc([shortSimParams.neighbor_radius],0))
    
    print("Running genetic algo:")
    num_generations=15
    with tqdm(total=num_generations) as pbar:
        ga_instance = pygad.GA(
            num_generations=num_generations,
            num_parents_mating=6,
            fitness_func=fitnessFunc,
            sol_per_pop=15,
            num_genes=1,
            # mutation_type="adaptive",
            mutation_probability=1,
            gene_type=float,
            gene_space = [{'low': 0, 'high': 10}],#,{'low': 0, 'high': 10}, {'low': 0, 'high': 10},{'low': 0, 'high': 10}],
            on_generation=lambda _:pbar.update(1) #make progress bar work
        )
        ga_instance.run()
        solution, solution_fitness, solution_idx = ga_instance.best_solution(ga_instance.last_generation_fitness)
        print("\nParameters of the best solution : {solution}".format(solution=solution))
        print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))

        print("Running final sim:")

        ga_instance.plot_fitness()
