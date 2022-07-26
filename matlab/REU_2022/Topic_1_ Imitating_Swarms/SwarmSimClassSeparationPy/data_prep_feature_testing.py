import numpy as np
import sim_tools.sim as sim
from sim_tools.sim import motionConstraints
import sim_tools.media_export as export
import copy
import os

import imitation_tools.automation as automation
from models import linearSuperSet as lss
import plotly.express as px
import pandas as pd
from features.features import *
from models.featureCombo import FeatureCombo as fc
from models.featureComboEnv import FeatureComboEnv as fcE

params = sim.SimParams(
    num_agents=50,
    dt=0.05,
    overall_time=10,
    enclosure_size=15,
    init_pos_max=None,  # if None, then defaults to enclosure_size
    agent_max_vel=9,
    init_vel_max=2,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=3,
    periodic_boundary=False
)

if __name__ == '__main__':

    # no exports rn, just testing correlation and such
    shortSimParams = copy.deepcopy(params)
    print("Running short sims")
    shortSimParams.num_agents = 20
    # strong effect on learning separation
    shortSimParams.enclosure_size = 1.5*shortSimParams.num_agents
    shortSimParams.overall_time = 3
    shortSimParams.init_pos_max = shortSimParams.enclosure_size
    shortSimParams.agent_max_vel = 7

    social_features = [
        Cohesion(),
        Alignment(),
        SeparationInv2(),
    ]

    env_features = [
        Inertia(),
        RectangularBoundSteer(shortSimParams.enclosure_size,
                              shortSimParams.enclosure_size, params.neighbor_radius)
    ]
    soc_gains = np.array([1, 1, 1])
    env_gains = np.array([1, 1])

    orig = fcE(soc_gains,
               social_features, env_gains, env_features)
    controllers=[copy.deepcopy(orig) for i in range(params.num_agents)]

    learning_features_social={
        "coh": Cohesion(),
        "align": Alignment(),
        "sep": SeparationInv2(),
    }

    learning_features_env = {
        "inertia": Inertia(),
        "steer": RectangularBoundSteer(shortSimParams.enclosure_size,
                                       shortSimParams.enclosure_size,
                                       shortSimParams.neighbor_radius)
    }

    featureSlices = automation.runSimsForFeatures(
        controllers, learning_features_social, learning_features_env, num_sims=1000, params=shortSimParams, threads=10)
    print(featureSlices[0])
    print(featureSlices[0].env_features["inertia"])
    print(np.array(list(featureSlices[0].env_features.values())))

    x = []
    y = []
    for slice in featureSlices:
        if slice.motion_constrained or slice.boundary_constrained:
            continue
        s_f = np.array(list(slice.social_features.values()))
        e_f = np.array(list(slice.env_features.values()))
        x.append(np.concatenate((s_f, e_f),axis=0))
        y.append(slice.output_vel)  # - slice.last_vel)

    true_loss = 0
    comb_gains = np.concatenate((soc_gains,env_gains))
    for i in range(len(x)):
        v_pred = np.dot(x[i].transpose(), comb_gains)
        # v_pred = x[i][0]*true_gains[0] + x[i][1]*true_gains[1] + x[i][2]*true_gains[2]
        v_actual = y[i]
        # if (np.sum(np.abs(v_pred-v_actual)) >= 1):
            # print("Pred(true gains)", v_pred, "Actual", v_actual)
            # print("Features", x[i])
        true_loss += np.sum(np.abs(v_pred-v_actual))  # sign issues sometimes
    print("True loss on avg", true_loss/len(x))

    y_pred_true = np.array(
        [np.dot(x_i.transpose(), np.array(comb_gains)) for x_i in x])
    # print(y_pred_true)
    y_pred_true_vx = y_pred_true[:, 0]
    y_pred_true_vy = y_pred_true[:, 1]
    # print("y_pred_true_vx",y_pred_true_vx)
    # print("y_pred_true_vy",y_pred_true_vy)

    y = np.array(y)
    y_vx = y[:, 0]
    y_vy = y[:, 1]

    direction_df = pd.DataFrame({
        "x": np.concatenate([y_pred_true_vx, y_vx]),
        "y": np.concatenate([y_pred_true_vy, y_vy]),
        "type": np.concatenate([["predicted"]*len(y_pred_true_vx), ["actual"]*len(y_vx)])
    })

    # plot with direction
    px.scatter(direction_df, x="x", y="y", color="type",
               title="Predicted(Using true gains) vs Actual").show()

    # #difference overall
    # y_mag = np.array([np.linalg.norm(y_i) for y_i in y])
    # diff = y_pred_true - y
    # diff_mag = np.linalg.norm(diff,axis=1)
    # px.scatter(x=np.arange(len(diff_mag)),y=diff_mag,title="Difference vs index").show()

    # # velocity mag per agent(check for capping)
    # px.scatter(x=np.arange(len(y_mag)),y=y_mag,title="True V_mag vs index").show()

    # #3d graph to see linearity
    # px.scatter_3d(x=x_flat[:,0],y=x_flat[:,1],z=y_flat,title="coh x align x v_out",size_max=5,opacity=.7).show()

    # #get rid of cycling(there is a better way)
    # x_flat = x_flat[:int(len(x_flat)/10)]
    # # #and most certainly a better way to plot collinearity

    # # # he had a collinearity graph
    # features_df = pd.DataFrame({
    #     "x":np.tile(np.arange(len(x_flat)),5),
    #     "y":np.concatenate([x_flat[:,0],x_flat[:,1],x_flat[:,2],x_flat[:,3],x_flat[:,4]]),
    #     "type":np.repeat(["cohesion","alignment","separation","steer_to_avoid","rotation"],len(x_flat))
    #     })
    # px.line(features_df,x="x",y="y",color="type",title="Features").show()
