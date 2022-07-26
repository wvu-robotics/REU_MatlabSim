from imitation_tools.data_prep import posVelSlice, featureSlice, toFeatureSlices
from sim_tools.sim import SimParams, motionConstraints
import numpy as np
from sklearn.linear_model import LinearRegression as lr
from sklearn.linear_model import ElasticNetCV as en
from scipy import optimize, signal
import pygad
import copy
from tqdm import tqdm
import plotly.express as px


def learnMotionConstraints(posVelSlices: list, params: SimParams, verbose=True) -> dict:
    print("Learning motion constraints:")
    # consider different kinds of motion models
    max_vel = 0
    max_accel = np.inf  # need to work on learning these
    max_turn_rate = np.inf  # need to work on learning

    all_vels = []

    for slice in (tqdm(posVelSlices) if verbose else posVelSlices):
        # if max_vel < np.max(np.abs(slice.vel)):
        #     max_vel = np.max(np.abs(slice.vel))
        all_vels.append(np.linalg.norm(slice.next_vel, axis=1))
        # print(all_vels[-1])

        accel = (slice.next_vel - slice.vel)/params.dt
        if max_accel < np.max(np.abs(accel)):
            max_accel = np.max(np.abs(accel))
        # turn rate stuff is a bit borked

    all_vels = np.array(all_vels)
    all_vels = all_vels.transpose()

    agent0Vels = copy.deepcopy(all_vels[0])

    # agent0Vels = agent0Vels[agent0Vels>4]

    # px.line(x=np.arange(len(agent0Vels)),
    #         y=agent0Vels, title="Agent 0 Vels").show()

    # px.line(x=np.arange(len(all_vels[50])),
    #         y=all_vels[50], title="Agent 50 Vels").show()

    # cutoff = 0.1 # Change this parameter from 0 to 1
    # sos = signal.butter(2, cutoff, output='sos')

    # agent0Vels_filtered = signal.sosfilt(sos,agent0Vels,axis=0)

    # px.line(x=np.arange(len(agent0Vels)),
    #     y=agent0Vels_filtered, title="Agent 0 Vels Filtered").show()

    all_vels_flat = all_vels.flatten()
    # # print(all_vels_flat.shape)
    # all_vels_flat = np.sort(all_vels_flat)

    # all_plot = px.line(x=np.arange(len(all_vels_flat)),
    #                    y=all_vels_flat, title="All Vels")
    # all_plot.show()

    # all_plot.write_image("all_vels.png")

    # throw out top 1%, average 10%
    perc1 = int(0.01*len(all_vels_flat))
    perc5 = int(0.05*len(all_vels_flat))

    tops = all_vels_flat[-perc5:-perc1]
    max_vel = np.max(tops)

    return {"max_vel": max_vel, "max_accel": max_accel, "max_turn_rate": max_turn_rate}


def dataForLinReg(featureSlices: list, params: SimParams, verbose=True):
    x_flat = []
    y_flat = []
    for slice in (tqdm(featureSlices) if verbose else featureSlices):
        if slice.motion_constrained or slice.boundary_constrained:
            continue

        s_f = np.array(list(slice.social_features.values()))
        e_f = np.array(list(slice.env_features.values()))
        x = np.concatenate((s_f, e_f), axis=0)

        x_flat.append(x[:, 0])
        y_flat.append(slice.output_vel[0])  # -slice.last_vel[0])
        x_flat.append(x[:, 1])
        y_flat.append(slice.output_vel[1])  # -slice.last_vel[1])
    return np.array(x_flat), np.array(y_flat)

# used in GA for neighbor radius


def fitnessLinearReg(posVelSlices: list, params: SimParams, learning_features: dict):
    def fitness(radius, sol_id):
        # only do genetic with on the neighbor radius, with fitness doing a linear regression
        localParams = copy.deepcopy(params)
        localParams.neighbor_radius = radius[0]

        # hrm maybe should be constraining motion more
        # I might migrate to new interface for speed
        agentSlices = toFeatureSlices(
            posVelSlices, learning_features, localParams, verbose=False)

        if len(agentSlices) == 0:
            return -np.inf

        # estimate boids gains
        # prep for linear regression
        # could be renested for motion/boundary constraints to avoid recalc
        X, y = dataForLinReg(agentSlices, localParams, verbose=False)
        if len(X) == 0:
            return -np.inf

        X = np.array(X)
        y = np.array(y)
        # fit linear regression
        # reg = lr()
        # reg.fit(X,y)
        model = en(cv=10).fit(X, y)
        # print("Data ",len(X))
        est_gains = model.coef_

        loss = 0.0
        for i in range(len(X)):
            v_pred = np.dot(est_gains.transpose(), X[i])
            v_actual = y[i]
            loss += np.sum(np.abs(v_pred-v_actual))

        # low radius penalty
        radPenalty = len(agentSlices)/(len(posVelSlices) * params.num_agents)

        return -loss/len(agentSlices) - radPenalty  # normalized between runs
    return fitness

# involves first pass linear regression-might be better to return gains from that at some point


def learnNeighborRadius(posVelSlices: list, params: SimParams, learning_features: dict, verbose=True) -> int:
    fitnessFunc = fitnessLinearReg(posVelSlices, params, learning_features)

    if verbose:
        print("Running genetic algorithm to learn radius:")
    num_generations = 10
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
                # ,{'low': 0, 'high': 10}, {'low': 0, 'high': 10},{'low': 0, 'high': 10}],
                gene_space=[{'low': 0, 'high': 10}],
                on_generation=lambda _: pbar.update(
                    1)  # make progress bar work
            )
            ga_instance.run()
            solution, solution_fitness, solution_idx = ga_instance.best_solution(
                ga_instance.last_generation_fitness)
            print("\nParameters of the best solution : {solution}".format(
                solution=solution))
            print("Fitness value of the best solution = {solution_fitness}".format(
                solution_fitness=solution_fitness))
            return solution[0]
    else:
        ga_instance = pygad.GA(
            num_generations=num_generations,
            num_parents_mating=4,
            fitness_func=fitnessFunc,
            sol_per_pop=8,
            num_genes=1,
            # mutation_type="adaptive",
            mutation_probability=1,
            gene_type=float,
            # ,{'low': 0, 'high': 10}, {'low': 0, 'high': 10},{'low': 0, 'high': 10}],
            gene_space=[{'low': 0, 'high': 10}],
        )
        ga_instance.run()
        solution, solution_fitness, solution_idx = ga_instance.best_solution(
            ga_instance.last_generation_fitness)
        return solution

# creating seperate in case we ever want to return to elastic net/lasso


def learnGainsLinearRegression(featureSlices: list, params: SimParams, soc_features: dict, env_features:dict,verbose=True) -> np.ndarray:
    # ignores motion constrained data
    if verbose:
        print("Learning gains using linear regression:")

    x_flat, y_flat = dataForLinReg(featureSlices, params, verbose)

    if verbose:
        print("Data length for linear regression", len(x_flat))
    if len(x_flat) == 0:
        return np.zeros(len(soc_features)+len(env_features))

    model = lr().fit(np.array(x_flat), np.array(y_flat))
    return model.coef_

# used in NL optimization, for gains


def sliceBasedLoss(featureSlices, params: SimParams):
    def loss(linear_gains):
        linear_gains = np.array(linear_gains)
        # print("Gains",est_gains)
        loss = 0.0
        for slice in featureSlices:
            if slice.boundary_constrained:
                continue

            s_f = np.array(list(slice.social_features.values()))
            e_f = np.array(list(slice.env_features.values()))
            x = np.concatenate((s_f, e_f), axis=0)
            vel_pred = np.dot(x.transpose(), linear_gains)  # + slice.last_vel
            vel_pred = motionConstraints(vel_pred, slice.last_vel, params)
            v_actual = slice.output_vel
            err = (vel_pred - v_actual)
            # could change to MSE, but I like accounting for direction better
            loss += np.linalg.norm(err)
        if len(x) == 0:
            return np.inf
        return loss / len(x)  # normalized between runs

    return loss

# could add an intermediary function/option to only optimize selected features


def learnGainsNonLinearOptimization(featureSlices: list, params: SimParams, guess: np.ndarray, maxSample: int = 5000, verbose=True) -> np.ndarray:
    np.random.shuffle(featureSlices)  # since we subsample the data
    loss_fn = sliceBasedLoss(
        featureSlices[:min(len(featureSlices), maxSample)], params)
    
    # maybe pass more options up the line
    # should maybe embed a 
    solution = optimize.minimize(loss_fn, (guess), method='SLSQP')
    return np.array(solution.x)
