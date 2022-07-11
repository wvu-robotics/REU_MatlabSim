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
    
    true_gains = np.array([1,3,1,0,1])

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


    featureSlices = automation.runSimsForFeatures(controllers,learning_features,num_sims=500,params=shortSimParams)
    # """print(featureSlices[0])
    # print(featureSlices[0].features["coh"])
    # print(np.array(list(featureSlices[0].features.values())))"""


    #ignore saturated data for this one
    x_flat = []
    y_flat = []
    for slice in featureSlices:
        if slice.motion_constrained or slice.boundary_constrained:
            continue
        f = slice.features
        x_flat.append(np.array(list(slice.features.values()))[:,0])
        y_flat.append(slice.output_vel[0] - slice.last_vel[0])
        x_flat.append(np.array(list(slice.features.values()))[:,1])
        y_flat.append(slice.output_vel[1] - slice.last_vel[1])

    true_loss = 0
    for i in range(len(x_flat)):
        v_pred = np.dot(x_flat[i].transpose(),true_gains)
        # v_pred = x[i][0]*true_gains[0] + x[i][1]*true_gains[1] + x[i][2]*true_gains[2]
        v_actual = y_flat[i]
        true_loss += np.sum(np.abs(v_pred-v_actual))
    print("True loss on avg, Flat, no saturation",true_loss/len(x_flat))

    #  doing true loss with constraints means I need to include last_vels
    # this loss is higher, which makes sense
    true_loss = 0
    for slice in featureSlices:
        if slice.boundary_constrained:
            continue
        x = np.array(list(slice.features.values()))
        v_pred = np.dot(x.transpose(),true_gains)+slice.last_vel
        v_pred = sim.motionConstraints(v_pred,slice.last_vel,params=shortSimParams)
        v_actual = slice.output_vel
        true_loss += np.sum(np.abs(v_pred-v_actual))
    print("True loss on avg, 2D,saturated: ",true_loss/len(x))


    print("First pass linear regression: ")
    reg = lr().fit(x_flat,y_flat) 

    print("R^2: ",reg.score(x_flat,y_flat))

    # #create some visuals with the imitated swarm
    #     #maybe do an imposter(sus) pretending to be within the original swarm
    linear_gains = reg.coef_.tolist()
    print("Linear gains",linear_gains)




    def sliceBasedFitness(featureSlices):
        def fitness(linear_gains):
            linear_gains = np.array(linear_gains)
            # print("Gains",est_gains)
            loss = 0.0
            for slice in featureSlices:
                if slice.boundary_constrained:
                    continue
                x = np.array(list(slice.features.values()))
                vel_pred = np.dot(x.transpose(),linear_gains) + slice.last_vel
                vel_pred = sim.motionConstraints(vel_pred, slice.last_vel, params)
                v_actual = slice.output_vel
                err = (vel_pred - v_actual)
                loss += np.linalg.norm(err)  # could change to MSE, but I like accounting for direction better
            if len(x) == 0:
                return np.inf
            return loss / len(x)  # normalized between runs

        return fitness

    # getting rid of filter for rn, I know pereira wanted it gone
    # filter = np.bitwise_not(np.isclose(np.array(linear_gains),0,atol=0.005))
    # print("Filter",filter)
    # filter_same_dim = np.tile(filter,len(x))

    # x_filtered = np.extract(filter_same_dim,x).reshape(len(x),-1) #let non-linear only learn off of features with non-zero gains
    # linear_gains_filtered = np.extract(filter,linear_gains)
    np.random.shuffle(featureSlices)

    fitness_function = sliceBasedFitness(featureSlices[:100])
    x_saturated = []
    for slice in featureSlices:
        if slice.boundary_constrained:
            continue
        x_saturated.append(np.array(list(slice.features.values())))

    #if np.all(linear_gains) == True:
    solution = optimize.minimize(fitness_function,(linear_gains), method='SLSQP')
    nl_gains = np.array(solution.x)

    # adding zeros back into output gains
    sol_gains = [] #can't think of a shorter way to write this rn
    sol_gains = nl_gains.tolist()
    # nl_cursor = 0
    # for bool in filter:
    #     if bool:
    #         sol_gains.append(nl_gains[nl_cursor])
    #         nl_cursor += 1
    #     else:
    #         sol_gains.append(0)

    print("Solution gains after NL step",sol_gains)


    # print("Parameters of the best solution : {params}".format(params=solution.x))
    #else:
        #zero_ind = np.where(linear_gains == 0)[0]
        #nonzero_gains = linear_gains[linear_gains != 0]
        #for i in range (len(linear_gains)):
            
        #solution = optimize.minimize(fitness_function,(linear_gains), method='SLSQP')
        #print("Parameters of the best solution : {params}".format(params=solution.x))
    
    print("Running imitated visual: ")
    controllers_imitated = [fc(sol_gains, (list(learning_features.values()))) for i in
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
    imitated_agents = [fc(sol_gains, list(learning_features.values())) for i in
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