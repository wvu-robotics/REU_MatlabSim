from imitation_tools.data_prep import posVelSlice,featureSlice,toFeatureSlices
from sim_tools.sim import SimParams,motionConstraints
import numpy as np
from sklearn.linear_model import LinearRegression as lr
from scipy import optimize
import pygad
import copy
import tqdm

def learnMotionConstraints(posVelSlices:list(posVelSlice),params:SimParams,verbose=True)->dict:
    print("Learning motion constraints:")
    # consider different kinds of motion models
    max_vel = 0
    max_accel = 0
    max_turn_rate = 0

    for slice in (tqdm(posVelSlices) if verbose else posVelSlices):
        if max_vel < np.max(np.abs(slice.vel)):
            max_vel = np.max(np.abs(slice.vel))
        accel = (slice.next_vel - slice.vel)/params.dt
        if max_accel < np.max(np.abs(accel)):
            max_accel = np.max(np.abs(accel))
        # turn rate stuff is a bit borked


    return {"max_vel":max_vel, "max_accel":max_accel, "max_turn_rate":max_turn_rate}

def dataForLinReg(featureSlices:list(featureSlice),params:SimParams,verbose=True):
    x_flat = []
    y_flat = []
    for slice in (tqdm(featureSlices) if verbose else featureSlices):
        if slice.motion_constrained or slice.boundary_constrained:
            continue
        f = slice.features
        x_flat.append(np.array(list(f.values()))[:,0])
        y_flat.append(slice.output_vel[0]-slice.last_vel[0])
        x_flat.append(np.array(list(f.values()))[:,1])
        y_flat.append(slice.output_vel[1]-slice.last_vel[1])
    return np.array(x_flat),np.array(y_flat)

#used in GA for neighbor radius
def fitnessLinearReg(posVelSlices:list(posVelSlice),params:SimParams,learning_features:dict):
        def fitness(radius,sol_id):
            # only do genetic with on the neighbor radius, with fitness doing a linear regression
            localParams = copy.deepcopy(params)
            localParams.neighbor_radius = radius[0]

            # hrm maybe should be constraining motion more
            # I might migrate to new interface for speed
            agentSlices = toFeatureSlices(posVelSlices,learning_features,localParams,verbose=False)
            
            if len(agentSlices) == 0:
                return -np.inf
            
            # estimate boids gains
            #prep for linear regression
            X,y = dataForLinReg(agentSlices,localParams,verbose=False)
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

            loss = 0.0
            for i in range(len(X)):
                v_pred = np.dot(est_gains.transpose(),X[i])
                v_actual = y[i]
                loss += np.sum(np.abs(v_pred-v_actual))

            return -loss/len(agentSlices) #normalized between runs
        return fitness

# involves first pass linear regression-might be better to return gains from that at some point
def learnNeighborRadius(posVelSlices:list(posVelSlice),params:SimParams,learning_features:dict,verbose=True)->int:
    fitnessFunc = fitnessLinearReg(posVelSlices,params,learning_features)

    if verbose: print("Running genetic algorithm to learn radius:")
    num_generations = 15
    if verbose:
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
            return solution[0]
    else:
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
            )
        ga_instance.run()
        solution, solution_fitness, solution_idx = ga_instance.best_solution(ga_instance.last_generation_fitness)
        return solution

# creating seperate in case we ever want to return to elastic net/lasso
def learnGainsLinearRegression(featureSlices:list(featureSlice),params:SimParams,verbose=True)->np.ndarray:
    #ignores motion constrained data
    if verbose: print("Learning gains using linear regression:")

    x_flat,y_flat = dataForLinReg(featureSlices,params,verbose)

    model = lr().fit(np.array(x_flat),np.array(y_flat))
    return model.coef_

# used in NL optimization, for gains
def sliceBasedLoss(featureSlices,params:SimParams):
    def loss(linear_gains):
        linear_gains = np.array(linear_gains)
        # print("Gains",est_gains)
        loss = 0.0
        for slice in featureSlices:
            if slice.boundary_constrained:
                continue
            x = np.array(list(slice.features.values()))
            vel_pred = np.dot(x.transpose(),linear_gains) + slice.last_vel
            vel_pred = motionConstraints(vel_pred, slice.last_vel, params)
            v_actual = slice.output_vel
            err = (vel_pred - v_actual)
            loss += np.linalg.norm(err)  # could change to MSE, but I like accounting for direction better
        if len(x) == 0:
            return np.inf
        return loss / len(x)  # normalized between runs

    return loss

# could add an intermediary function/option to only optimize selected features
def learnGainsNonLinearOptimization(featureSlices:list(featureSlice),params:SimParams,guess:np.ndarray,maxSample:int=5000,verbose=True)->np.ndarray:
    np.random.shuffle(featureSlices) # since we subsample the data
    loss_fn = sliceBasedLoss(featureSlices[:min(len(featureSlices),maxSample)],params)
    solution = optimize.minimize(loss_fn,(guess), method='SLSQP') #maybe pass more options up the line
    return np.array(solution.x)