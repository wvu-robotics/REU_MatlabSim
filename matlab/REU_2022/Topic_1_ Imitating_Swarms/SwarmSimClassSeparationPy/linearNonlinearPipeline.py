import numpy as np
import sim_tools.sim as sim
from  sim_tools.sim import motionConstraints
import sim_tools.media_export as export
import copy
import os

import imitation_tools.automation as automation
from models import linearSuperSet as lss
import plotly.express as px
import pandas as pd
from features.features import *
from models.featureCombo import FeatureCombo as fc
from scipy import optimize

from sklearn.linear_model import (
    Lasso,
    Ridge as rr,
    ElasticNet,
    ElasticNetCV,
    LinearRegression as lr,
    TheilSenRegressor,
    HuberRegressor
)
from sklearn.feature_selection import (
    RFECV,
    RFE
)
from sklearn.pipeline import Pipeline


params = sim.SimParams(
    num_agents=50,
    dt=0.05,
    overall_time = 15,
    enclosure_size = 15,
    init_pos_max = None, #if None, then defaults to enclosure_size
    agent_max_vel=7,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=3,
    periodic_boundary=False
    )

if __name__ == '__main__':
    orig_features = [
        Cohesion(),
        Alignment(), 
        SeparationInv2(),
        SteerToAvoid(params.neighbor_radius/4,params.neighbor_radius),
        Rotation()
    ]
    
    true_gains = np.array([1,1,1,0,1])

    orig = fc(true_gains,orig_features)
    controllers = [copy.deepcopy(orig) for i in range(params.num_agents)]

    #initial simulation
    print("First sim and export: ")
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

    if not os.path.exists("linearNonlinearOutput"):
        os.makedirs("linearNonlinearOutput")

    export.export(export.ExportType.GIF,"linearNonlinearOutput/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)

    #shortsim params
    shortSimParams = copy.deepcopy(params)
    print("Running short sims")
    shortSimParams.num_agents = 20
    shortSimParams.enclosure_size = 20 #strong effect on learning separation
    shortSimParams.overall_time = 2
    shortSimParams.init_pos_max = shortSimParams.enclosure_size
    shortSimParams.agent_max_vel = 7

    learning_features = {
        "coh": Cohesion(),
        "align": Alignment(),
        "sep": SeparationInv2(),
        "steer": SteerToAvoid(params.neighbor_radius/4,params.neighbor_radius),
        "rot": Rotation()
    }


    featureSlices = automation.runSimsForFeatures(controllers,learning_features,num_sims=500,params=shortSimParams,ignoreMC=True)
    """print(featureSlices[0])
    print(featureSlices[0].features["coh"])
    print(np.array(list(featureSlices[0].features.values())))"""

    x = []
    y = []
    for slice in featureSlices:
        f = slice.features
        x.append(np.array(list(slice.features.values()))[:,0])
        y.append(slice.output_vel[0] - slice.last_vel[0])
        x.append(np.array(list(slice.features.values()))[:,1])
        y.append(slice.output_vel[1] - slice.last_vel[1])
    

    print("First pass linear regression: ")
    reg = ElasticNetCV(cv=10).fit(x,y) 

    print("R^2: ",reg.score(x,y))

    # #create some visuals with the imitated swarm
    #     #maybe do an imposter(sus) pretending to be within the original swarm
    linear_gains = reg.coef_.tolist()
    print(linear_gains)

    true_loss = 0
    for i in range(len(x)):
        v_pred = np.dot(x[i].transpose(),true_gains)
        # v_pred = x[i][0]*true_gains[0] + x[i][1]*true_gains[1] + x[i][2]*true_gains[2]
        v_actual = y[i]
        true_loss += np.sum(np.abs(v_pred-v_actual))
    print("True loss on avg: ",true_loss/len(x))

    print("Running imitated visual: ")

    def sliceBasedFitness(x):
        def fitness(linear_gains):
            linear_gains = np.array(linear_gains)
            # print("Gains",est_gains)
            loss = 0.0
            for i in range(len(x)):
                
                vel_pred = np.dot(x[i].transpose(),linear_gains)
                #vel_pred = sim.motionConstraints(vel_pred, slice.last_vel, params)
                err = (vel_pred - y[i])
                loss += np.linalg.norm(err)  # could change to MSE, but I like accounting for direction better
            if len(x) == 0:
                return np.inf
            return loss / len(x)  # normalized between runs

        return fitness

    fitness_function = sliceBasedFitness(x)
    solution = optimize.minimize(fitness_function,(linear_gains[0], linear_gains[1], linear_gains[2], linear_gains[3], linear_gains[4]), method='SLSQP')
    print("Parameters of the best solution : {params}".format(params=solution.x))

    controllers_imitated = [fc(linear_gains, (list(learning_features.values()))) for i in
                            range(params.num_agents)]
    for controller in controllers_imitated:
        controller.setColor('red')
    agentPositions_imitated, agentVels_imitated = sim.runSim(controllers_imitated, params, progress_bar=True, initial_positions=agentPositions[0], initial_velocities=agentVels[0])
    export.export(export.ExportType.GIF, "linearNonlinearOutput/Imitated", agentPositions_imitated, agentVels_imitated,
                  controllers=controllers_imitated, params=params, progress_bar=True)
    
    # now create some hybrid visualizations
    print("Running hybrid visual: ")
    # some parameters for hybrid visualization
    mix_factor = 0.5
    params.num_agents = 50
    params.enclosure_size = 20
    params.overall_time = 15
    params.init_pos_max = params.enclosure_size
    params.agent_max_vel = 7

    original_agents = [fc(true_gains, list(learning_features.values())) for i in range(int(params.num_agents * mix_factor))]
    for controller in original_agents:
        controller.setColor("rgb(99, 110, 250)")
    imitated_agents = [fc(linear_gains, list(learning_features.values())) for i in
                       range(int(params.num_agents * (1 - mix_factor)))]
    for controller in imitated_agents:
        controller.setColor("red")

    all_controllers = original_agents + imitated_agents

    agentPositions_hybrid, agentVels_hybrid = sim.runSim(all_controllers, params, progress_bar=True)
    export.export(export.ExportType.GIF, "linearNonlinearOutput/Hybrid", agentPositions_hybrid, agentVels_hybrid,
                  controllers=all_controllers, params=params, vision_mode=False, progress_bar=True)


"""
    y_pred_true = np.array([np.dot(x_i.transpose(),np.array(true_gains)) for x_i in x])
    # print(y_pred_true)
    y_pred_true_vx = y_pred_true[:,0]
    y_pred_true_vy = y_pred_true[:,1]
    # print("y_pred_true_vx",y_pred_true_vx)
    # print("y_pred_true_vy",y_pred_true_vy)


    y = np.array(y)
    y_vx= y[:,0]
    y_vy= y[:,1]

    direction_df = pd.DataFrame({
        "x":np.concatenate([y_pred_true_vx,y_vx]),
        "y":np.concatenate([y_pred_true_vy,y_vy]),
        "type":np.concatenate([["predicted"]*len(y_pred_true_vx),["actual"]*len(y_vx)])
        })
"""   