from dataclasses import dataclass
from pipeline_tools.learning import learnMotionConstraints, learnNeighborRadius, learnGainsLinearRegression, learnGainsNonLinearOptimization, sliceBasedLoss
from sim_tools import sim
from sim_tools import media_export as export
from imitation_tools import data_prep
from features.features import *
from pipeline_tools.metrics import plotMetrics, kStepPredictionErrorNormalized
from pipeline_tools.transformations import gaussianNoisePositions

from models.featureComboEnv import FeatureComboEnv as fcE
from tqdm import tqdm
import numpy as np
import copy
import os


@dataclass
class trainingCriteria:
    num_agents: int
    enclosure_size: int
    overall_time: int
    init_vel_max: int
    num_sims: int

# run simulation, learn imitation from it, output efficacy of imitation and recreation


def runPipelineArtificialSwarm(params: sim.SimParams, orig_soc_features, orig_env_features, in_gains, learn_s_f, learn_e_f, training: trainingCriteria, out_file_dir):
    # INITIAL GENERATION
    true_gains = in_gains
    orig_controller = fcE(true_gains[:len(orig_soc_features)], orig_soc_features, true_gains[len(
        orig_soc_features):], orig_env_features)
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
        positions, vels = sim.runSim(
            orig_controllers, trainingParams, progress_bar=False)
        positions = gaussianNoisePositions(
            positions, 0.05*params.agent_max_vel*params.dt)
        posVelSlices.extend(data_prep.toPosVelSlices(positions, params))

    # learn from initial simulation
    learnedParams = sim.SimParams()
    learnedParams.overall_time = params.overall_time
    learnedParams.dt = trainingParams.dt
    learnedParams.num_agents = trainingParams.num_agents
    learnedParams.enclosure_size = trainingParams.enclosure_size

    learnedMCs = learnMotionConstraints(posVelSlices, learnedParams)
    print("Learned MCs", learnedMCs)
    learnedParams.agent_max_vel = learnedMCs["max_vel"]

    # common practice to run GA on smaller sampling just for the sake of reasonable speed
    np.random.shuffle(posVelSlices)
    radius = learnNeighborRadius(
        posVelSlices[:500], learnedParams, learn_s_f, learn_e_f)
    print("Learned Radius", radius)
    # radius = params.neighbor_radius #deactivate radius learning for a second
    learnedParams.neighbor_radius = radius

    featureSlices = data_prep.toFeatureSlices(
        posVelSlices, learn_s_f, learn_e_f, learnedParams)

    linear_gains = learnGainsLinearRegression(
        featureSlices, learnedParams, learn_s_f, learn_e_f)
    print("Linear gains", linear_gains)

    improved_gains = learnGainsNonLinearOptimization(
        featureSlices, learnedParams, guess=linear_gains, maxSample=5000)
    print("Improved Gains", improved_gains)

    # average 1 step trajectory loss, same as used in NL opt
    loss = sliceBasedLoss(featureSlices, learnedParams)(improved_gains)

    # run output simulation with learned gains
    learned_controller = fcE(improved_gains[:len(
        learn_s_f)], list(learn_s_f.values()), improved_gains[len(learn_s_f):], list(learn_e_f.values()))
    learned_controllers = [copy.deepcopy(learned_controller)
                           for i in range(params.num_agents)]

    imitSimPos, imitSimVels = sim.runSim(learned_controllers, learnedParams,
                                         initial_positions=initSimPos[0], initial_velocities=initSimVels[0], progress_bar=False)

    # export results
    np.set_printoptions(precision=3)
    orig_gain_str = "o[" + ",".join("%.2f" % x for x in true_gains)+"]"
    learned_gain_str = "l[" + ",".join("%.2f" % x for x in improved_gains)+"]"
    orig_radius_str = "r_o" + "%.2f" % params.neighbor_radius
    learned_radius_str = "r_l" + "%.2f" % radius

    orig_max_vel_str = "v_o" + "%.2f" % params.agent_max_vel
    learned_max_vel_str = "v_l" + "%.2f" % learnedParams.agent_max_vel

    # create output directory if not found
    if not os.path.exists(out_file_dir):
        os.makedirs(out_file_dir)

    np.set_printoptions(precision=5)
    loss_str = "loss:" + "%.3f" % loss
    # pvs_orig = data_prep.toPosVelSlices(initSimPos, params)

    # step1_error = kStepPredictionErrorNormalized(
    #     pvs_orig, learned_controller, learnedParams, 1)
    # step5_error = kStepPredictionErrorNormalized(
    #     pvs_orig, learned_controller, learnedParams, 5)
    # step10_error = kStepPredictionErrorNormalized(
    #     pvs_orig, learned_controller, learnedParams, 10)

    # print("1 step error", step1_error)
    # print("5 step error", step5_error)
    # print("10 step error", step10_error)

    # print("Truth test:")

    # step1_true_error = kStepPredictionErrorNormalized(pvs_orig, orig_controller, params, 1)
    # step5_true_error = kStepPredictionErrorNormalized(pvs_orig, orig_controller, params, 5)
    # step10_true_error = kStepPredictionErrorNormalized(pvs_orig, orig_controller, params, 10)

    # print("1 step true error", step1_true_error)
    # print("5 step true error", step5_true_error)
    # print("10 step true error", step10_true_error)

    # loss_str = "1s:%.3f" % (
    #     step1_error) + " 5s:%.3f" % (step5_error) + " 10s:%.3f" % (step10_error)

    # write file outputs
    o_file_name = out_file_dir+"/"+orig_radius_str+orig_max_vel_str+orig_gain_str
    l_file_name = out_file_dir+"/"+learned_radius_str+orig_radius_str + \
        learned_max_vel_str+orig_max_vel_str+learned_gain_str+orig_gain_str+loss_str

    # export first 5s of trajectories
    export.exportTrajectories(o_file_name, initSimPos, params, 0, 5)
    export.exportTrajectories(l_file_name, imitSimPos, params, 0, 5)

    initPVS = data_prep.toPosVelSlices(initSimPos, params)
    imitPVS = data_prep.toPosVelSlices(imitSimPos, params)

    plotMetrics(out_file_dir+"graph", initPVS, imitPVS)

    np.savez(l_file_name, orig_positions=initSimPos, imit_positions=imitSimPos, learnedParams=learnedParams, params=params, orig_env_features=orig_env_features,
             orig_soc_features=orig_soc_features, learn_s_f=learn_s_f, learn_e_f=learn_e_f, orig_gains=true_gains, learned_gains=improved_gains, learned_controller=learned_controller,orig_controller=orig_controller)
