# import sys
# sys.path.append("..")
import numpy as np
from sqlalchemy import false
import sim_tools.sim as sim
import sim_tools.media_export as export
import copy
import os

import imitation_tools.automation as automation

from sklearn.linear_model import LinearRegression as lr
from sklearn.linear_model import Ridge as rr
from tqdm import tqdm
from dataclasses import dataclass #data class is like the python equivalent of a struct
from models import Boids as bo
from models import Dance
from models import linearSuperSet as lss
from scipy import optimize


params = sim.SimParams(
    num_agents=50,
    dt=0.01,
    overall_time = 10,
    enclosure_size = 15,
    init_pos_max = None, #if None, then defaults to enclosure_size
    agent_max_vel=5,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=3,
    periodic_boundary=False
    )

#consider adding velocity continuation in the void to Boids(so no need to crank inertia)
#mess around with inertia more, periodic bounds on the short sims(more realistic?)
#think more about adjusted initial conditions on the short sims

#also add fake things soon

if __name__ ==  '__main__':


    #run sim
    # print("Original agent slices")

    #ran first sim
    # can probably shorten constructor definitions

    cohesion_gain = 2
    align_gain = 1
    separation_gain = 2
    steer_to_avoid_gain = 0
    rotation_gain = 2
    inertia = 1

    true_gains = [cohesion_gain, align_gain, separation_gain, steer_to_avoid_gain, rotation_gain]

    orig_controller = lss.SuperSet(*true_gains)
    controllers = [ copy.deepcopy(orig_controller) for i in range(params.num_agents)]
    print("First sim and export")
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

    if not os.path.exists("TNCImitatorOutput"):
        os.makedirs("TNCImitatorOutput")

    export.export(export.ExportType.MP4,"TNCImitatorOutput/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)



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

    # posVelSlices = [posVelSlice(agentPositions[i],agentVels[i],agentVels[i+1]) for i in range(len(agentPositions)-1)]

    posVelSlices = []
    #run tons more short sims
    shortSimParams = copy.deepcopy(params)
    print("Running short sims")
    shortSimParams.num_agents = 20
    shortSimParams.enclosure_size = 1.5*shortSimParams.num_agents #strong effect on learning separation
    shortSimParams.overall_time = 2
    shortSimParams.init_pos_max = shortSimParams.enclosure_size
    shortSimParams.agent_max_vel = 10
    shortControllers = [copy.deepcopy(orig_controller) for i in range(shortSimParams.num_agents)]
    #colors = ["rgb(255, 0, 24)","rgb(255, 165, 44)","rgb(255, 255, 65)","rgb(0, 128, 24)","rgb(0, 0, 249)","rgb(134, 0, 125)","rgb(85, 205, 252)","rgb(247, 168, 184)"]

    agentSlices = automation.runSims(shortControllers,params=shortSimParams,num_sims=10,ignoreMC=True,
    export_info=[export.ExportType.MP4,"TNCImitatorOutput/ShortSims",50],threads=10)

    print("Num agent slices",len(agentSlices))

    x = []
    y = []

    #reshapes were being annoying, will rewrite better
    for slice in agentSlices:
        x.append(np.array([slice.cohesion[0],slice.alignment[0],slice.separation[0],slice.steer_to_avoid[0], slice.rotation[0]]))
        y.append(np.array(slice.output_vel[0]))
        x.append(np.array([slice.cohesion[1],slice.alignment[1],slice.separation[1],slice.steer_to_avoid[1], slice.rotation[1]]))
        y.append(np.array(slice.output_vel[1]))


    # print("True loss: ",true_loss)

    print("First pass linear regression: ")
    reg = lr(fit_intercept=False).fit(x,y) #lr = least squares regression, rr = ridge regression (l2 penalization)
    print("R^2: ",reg.score(x,y))

    # #create some visuals with the imitated swarm
    #     #maybe do an imposter(sus) pretending to be within the original swarm
    gains = reg.coef_.tolist()
    print("Coh: " + str(gains[0]) + ", Align: " + str(gains[1]) + ", Sep: " + str(gains[2]) + ", Steer: " + str(gains[3]) + ", Rot: " + str(gains[4]))

    print("Post Linear Regression: ")
    controllers = [lss.SuperSet(*gains) for i in range(params.num_agents)]
    agentPositions, agentVels = sim.runSim(controllers, params, progress_bar=True)


    export.export(export.ExportType.MP4, "TNCImitatorOutput/PostLRSim", agentPositions, agentVels, params=params,
                  vision_mode=False, progress_bar=True)

    posVelSlices = []
    # run tons more short sims
    shortSimParams = copy.deepcopy(params)
    print("Running short sims")
    shortSimParams.overall_time = 2
    shortSimParams.init_pos_max = shortSimParams.enclosure_size*1
    shortSimParams.dt = 0.01

    #agentSlices = automation.runSims(controllers, params=shortSimParams, num_sims=3, threads=4, ignoreMC=False)


    # sanity check, this should have 0 loss, anything else is either noise or bad measurements
    # this is a bit high and I don't like it
    # true_loss = 0
    # for slice in agentSlices:
    # gains_ex = true_gains[:3]
    # args = (slice.cohesion,slice.alignment,slice.separation)
    # print("Args",args)
    # print("Gains",gains_ex)
    # vel_pred = np.dot(args,gains_ex)
    # vel_pred = sim.motionConstraints(vel_pred,slice.last_vel,params)
    # err = vel_pred - slice.output_vel
    # print("err",err)
    # true_loss += np.linalg.norm(err)
    # true_loss/=len(agentSlices)
    # print("True loss(data deviation from original on average):",true_loss)

    # using function currying to make this a tad more useful



    # shuffle data and subsample it for optimization
    np.random.shuffle(agentSlices)
    agentSlicesSubsampled = agentSlices[:int(len(agentSlices)/10)]

    def sliceBasedFitness(agentSlices):
        def fitness(est_gains):
            est_gains = np.array(est_gains)
            # print("Gains",est_gains)
            loss = 0.0
            for slice in agentSlices:
                vel_pred = np.dot(est_gains.transpose(), np.array([slice.cohesion, slice.alignment, slice.separation, slice.steer_to_avoid, slice.rotation]))
                vel_pred = sim.motionConstraints(vel_pred, slice.last_vel, params)
                err = (vel_pred - slice.output_vel)
                loss += np.linalg.norm(err)  # could change to MSE, but I like accounting for direction better
            if len(agentSlices) == 0:
                return np.inf
            return loss / len(agentSlices)  # normalized between runs

        return fitness


    fitness_function = sliceBasedFitness(agentSlicesSubsampled)
    boundary = [(-1, 5), (-1, 5), (-1, 5), (-1, 5), (-5, 5)]
    #solution = optimize.differential_evolution(fitness_function, boundary, maxiter=50000000, x0=[gains[0], gains[1], gains[2], gains[3], gains[4]])
    solution = optimize.minimize(fitness_function,([gains[0], gains[1], gains[2], gains[3], gains[4]]), method='SLSQP', options={'ftol': 1e-08, 'iprint': 1, 'eps': 1.5e-12, 'maxiter': 10000})
    print("Parameters of the best solution : {params}".format(params=solution.x))

    controllers_imitated = [lss.SuperSet(solution.x[0], solution.x[1], solution.x[2], solution.x[3], solution.x[4]) for i in
                            range(params.num_agents)]
    for controller in controllers_imitated:
        controller.setColor('red')
    agentPositions_imitated, agentVels_imitated = sim.runSim(controllers_imitated, params, progress_bar=True)
    export.export(export.ExportType.MP4, "TNCImitatorOutput/Imitated", agentPositions_imitated, agentVels_imitated,
                  controllers=controllers_imitated, params=params, progress_bar=True)

    # now create some hybrid visualizations
    print("Running hybrid visual")
    # some parameters for hybrid visualization
    mix_factor = 0.6
    params.num_agents = 100
    params.enclosure_size = 20
    params.overall_time = 20
    params.init_pos_max = params.enclosure_size
    params.agent_max_vel = 7

    original_agents = [lss.SuperSet(*true_gains) for i in range(int(params.num_agents * mix_factor))]
    for controller in original_agents:
        controller.setColor("rgb(99, 110, 250)")
    imitated_agents = [lss.SuperSet(solution.x[0], solution.x[1], solution.x[2], solution.x[3], solution.x[4]) for i in
                       range(int(params.num_agents * (1 - mix_factor)))]
    for controller in imitated_agents:
        controller.setColor("red")

    all_controllers = original_agents + imitated_agents

    agentPositions_hybrid, agentVels_hybrid = sim.runSim(all_controllers, params, progress_bar=True)
    export.export(export.ExportType.MP4, "TNCImitatorOutput/Hybrid", agentPositions_hybrid, agentVels_hybrid,
                  controllers=all_controllers, params=params, vision_mode=False, progress_bar=True)
