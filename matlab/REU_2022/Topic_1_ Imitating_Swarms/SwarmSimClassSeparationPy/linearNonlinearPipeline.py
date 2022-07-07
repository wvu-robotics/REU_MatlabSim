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
    overall_time = 10,
    enclosure_size = 15,
    init_pos_max = None, #if None, then defaults to enclosure_size
    agent_max_vel=7,
    init_vel_max = 2,
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

    export.export(export.ExportType.GIF,"linearNonlinear/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)

    # no exports rn, just testing correlation and such
    shortSimParams = copy.deepcopy(params)
    print("Running short sims")
    shortSimParams.num_agents = 20
    shortSimParams.enclosure_size = 1*shortSimParams.num_agents #strong effect on learning separation
    shortSimParams.overall_time = 2
    shortSimParams.init_pos_max = shortSimParams.enclosure_size
    shortSimParams.agent_max_vel = 7

    learning_features = {
        "coh": Cohesion(),
        "align": Alignment(),
        "sep": SeparationInv2()
    }


    featureSlices = automation.runSimsForFeatures(controllers,learning_features,num_sims=500,params=shortSimParams,ignoreMC=True)
    """print(featureSlices[0])
    print(featureSlices[0].features["coh"])
    print(np.array(list(featureSlices[0].features.values())))"""

    x = []
    y = []
    for slice in featureSlices:
        f = slice.features
        x.append(np.array([f["coh"][0],f["align"][0],f["sep"][0]]))
        y.append(slice.output_vel[0] - slice.last_vel[0])
        x.append(np.array([f["coh"][1],f["align"][1],f["sep"][1]]))
        y.append(slice.output_vel[1] - slice.last_vel[1])
    

    print("First pass linear regression: ")
    reg = ElasticNetCV(cv=10).fit(x,y) 

    print("R^2: ",reg.score(x,y))

    # #create some visuals with the imitated swarm
    #     #maybe do an imposter(sus) pretending to be within the original swarm
    gains = reg.coef_.tolist()
    print(gains)

    true_loss = 0
    for i in range(len(x)):
        v_pred = np.dot(x[i].transpose(),true_gains)
        # v_pred = x[i][0]*true_gains[0] + x[i][1]*true_gains[1] + x[i][2]*true_gains[2]
        v_actual = y[i]
        true_loss += np.sum(np.abs(v_pred-v_actual))
    print("True loss on avg: ",true_loss/len(x))


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
    