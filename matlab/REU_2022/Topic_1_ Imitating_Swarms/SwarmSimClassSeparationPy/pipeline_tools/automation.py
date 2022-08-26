from dataclasses import dataclass
from pipeline_tools.learning import learnMotionConstraints, learnNeighborRadius, learnGainsLinearRegression, learnGainsNonLinearOptimization,sliceBasedLoss
from sim_tools import sim
from sim_tools import media_export as export
from imitation_tools import data_prep
from features.features import *

from models.featureComboEnv import FeatureComboEnv as fcE
from tqdm import tqdm
import numpy as np
import copy
import os


@dataclass
class trainingCriteria:
    num_agents:int
    enclosure_size:int
    overall_time:int
    init_vel_max:int
    num_sims:int

# run simulation, learn imitation from it, output efficacy of imitation and recreation
def runPipelineArtificialSwarm(params:sim.SimParams,orig_soc_features,orig_env_features,in_gains,learn_s_f,learn_e_f,training:trainingCriteria,out_file_dir):
    # INITIAL GENERATION
    true_gains = in_gains
    orig_controller = fcE(true_gains[:len(orig_soc_features)],orig_soc_features,true_gains[len(orig_soc_features):], orig_env_features)
    orig_controllers = [copy.deepcopy(orig_controller)
                        for i in range(params.num_agents)]

    initSimPos, initSimVels = sim.runSim(
        orig_controllers, params, progress_bar=True)
    
    trainingParams = copy.deepcopy(params)
    trainingParams.num_agents = training.num_agents
    trainingParams.enclosure_size = training.enclosure_size
    trainingParams.overall_time = training.overall_time
    trainingParams.init_vel_max = training.init_vel_max

    posVelSlices = []
    for i in range(training.num_sims):
        posVelSlices.append(sim.runSim(
            orig_controllers, trainingParams, progress_bar=True))


    # learn from initial simulation
    learnedParams = sim.SimParams()
    learnedParams.dt = trainingParams.dt
    learnedParams.num_agents = trainingParams.num_agents
    learnedParams.enclosure_size = trainingParams.enclosure_size

    learnedMCs = learnMotionConstraints(posVelSlices, learnedParams)
    print("Learned MCs", learnedMCs)
    learnedParams.agent_max_vel = learnedMCs["max_vel"]

    radius = learnNeighborRadius(
        posVelSlices, learnedParams, learn_s_f,learn_e_f)
    print("Learned Radius", radius)
    learnedParams.neighbor_radius = radius

    featureSlices = data_prep.toFeatureSlices(
        posVelSlices, learn_s_f,learn_e_f, learnedParams)

    linear_gains = learnGainsLinearRegression(
        featureSlices, learnedParams, learn_s_f,learn_e_f)
    print("Linear gains", linear_gains)

    improved_gains = learnGainsNonLinearOptimization(
        featureSlices, learnedParams, guess=linear_gains, maxSample=5000)
    print("Improved Gains", improved_gains)

    # average 1 step trajectory loss, same as used in NL opt
    loss = sliceBasedLoss(featureSlices, learnedParams)(improved_gains)

    # run output simulation with learned gains
    learned_controller = fcE(improved_gains[:len(orig_soc_features)],orig_soc_features,improved_gains[len(orig_soc_features):], orig_env_features)
    learned_controllers = [copy.deepcopy(learned_controller)
                            for i in range(params.num_agents)]

    imitSimPos, imitSimVels = sim.runSim(learned_controllers, learnedParams,
                                         initial_positions=initSimPos[0], initial_velocities=initSimVels[0], progress_bar=True)

    # export results
    np.set_printoptions(precision=3)
    orig_gain_str = "o[" +",".join(str(x) for x in true_gains)+"]"
    learned_gain_str = "l[" +",".join(str(x) for x in improved_gains)+"]"
    orig_radius_str = "r_o" + str(params.neighbor_radius)
    learned_radius_str = "r_l" + str(radius)

    orig_max_vel_str = "v_o" + str(params.agent_max_vel)
    learned_max_vel_str = "v_l" + str(learnedParams.agent_max_vel)

    # create output directory if not found
    if not os.path.exists(out_file_dir):
        os.makedirs(out_file_dir)

    np.set_printoptions(precision=5)
    loss_str = "loss:" + str(loss)

    # write file outputs
    o_file_name = out_file_dir+"/"+orig_radius_str+orig_max_vel_str+orig_gain_str
    l_file_name = out_file_dir+"/"+learned_radius_str+orig_radius_str+learned_max_vel_str+orig_max_vel_str+learned_gain_str+orig_gain_str

    # export first 5s of trajectories
    export.exportTrajectories(o_file_name,initSimPos,params,0,5)
    export.exportTrajectories(l_file_name,imitSimPos,params,0,5)

