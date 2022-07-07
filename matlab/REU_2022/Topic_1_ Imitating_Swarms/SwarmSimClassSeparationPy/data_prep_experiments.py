import numpy as np
import sim_tools.sim as sim
import sim_tools.media_export as export
import copy
import os

import imitation_tools.automation as automation
from models import linearSuperSet as lss
import plotly.express as px
import pandas as pd

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

    export.export(export.ExportType.GIF,"TNCImitatorOutput/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)

    posVelSlices = []
    #run tons more short sims
    shortSimParams = copy.deepcopy(params)
    print("Running short sims")
    shortSimParams.num_agents = 20
    shortSimParams.enclosure_size = 1.5*shortSimParams.num_agents #strong effect on learning separation
    shortSimParams.overall_time = 3
    shortSimParams.init_pos_max = shortSimParams.enclosure_size
    shortSimParams.agent_max_vel = 7
    shortControllers = [copy.deepcopy(orig_controller) for i in range(shortSimParams.num_agents)]
    #colors = ["rgb(255, 0, 24)","rgb(255, 165, 44)","rgb(255, 255, 65)","rgb(0, 128, 24)","rgb(0, 0, 249)","rgb(134, 0, 125)","rgb(85, 205, 252)","rgb(247, 168, 184)"]

    agentSlices = automation.runSims(shortControllers,params=shortSimParams,num_sims=100,ignoreMC=True,ignoreBC=True,
    export_info=[export.ExportType.GIF,"TNCImitatorOutput/ShortSims",50],threads=10)

    print("Num agent slices",len(agentSlices))

    x = []
    y = []
    x_flat = []
    y_flat = []

    #reshapes were being annoying, will rewrite better
    for slice in agentSlices:
        x.append(np.array([slice.cohesion,slice.alignment,slice.separation,slice.steer_to_avoid,slice.rotation]))
        y.append(np.array(slice.output_vel))
        x_flat.append(np.array([slice.cohesion[0],slice.alignment[0],slice.separation[0],slice.steer_to_avoid[0], slice.rotation[0]]))
        y_flat.append(np.array(slice.output_vel[0]))
        # x_flat.append(np.array([slice.cohesion[1],slice.alignment[1],slice.separation[1],slice.steer_to_avoid[1], slice.rotation[1]]))
        # y_flat.append(np.array(slice.output_vel[1]))

    print("x_length",len(x))
    x_flat = np.array(x_flat)
    y_flat = np.array(y_flat)

    true_loss = 0
    for i in range(len(x)):
        pred = np.dot(x[i].transpose(),np.array(true_gains))
        # if i<10:
        #     print(pred)
        #     print(y[i])
        true_loss += np.sum(np.abs(pred - y[i]))
    print("True loss on avg: ",true_loss/len(x))

    true_loss = 0
    for i in range(len(x_flat)):
        pred = cohesion_gain*x_flat[i][0] + align_gain*x_flat[i][1] + separation_gain*x_flat[i][2] + steer_to_avoid_gain*x_flat[i][3] + rotation_gain*x_flat[i][4]
        true_loss += np.sum(np.abs(pred - y_flat[i]))
    print("True loss on avg Flat: ",true_loss/len(x_flat))
    # makes sense different by 2, since slices are doubled in flat

    ##### Dimas-inspired plots

    # Original gains vs Actual
    # def mult_combine()
# x * np.array(true_gains)

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

    #plot with direction
    px.scatter(direction_df,x="x",y="y",color="type",title="Predicted(Using true gains) vs Actual").show()

    #difference overall
    y_mag = np.array([np.linalg.norm(y_i) for y_i in y])
    diff = y_pred_true - y
    diff_mag = np.linalg.norm(diff,axis=1)
    px.scatter(x=np.arange(len(diff_mag)),y=diff_mag,title="Difference vs index").show()

    # velocity mag per agent(check for capping)
    px.scatter(x=np.arange(len(y_mag)),y=y_mag,title="True V_mag vs index").show()
    
    #3d graph to see linearity
    px.scatter_3d(x=x_flat[:,0],y=x_flat[:,1],z=y_flat,title="coh x align x v_out",size_max=5,opacity=.7).show()


    #get rid of cycling(there is a better way)
    x_flat = x_flat[:int(len(x_flat)/10)]
    # #and most certainly a better way to plot collinearity

    # # he had a collinearity graph
    features_df = pd.DataFrame({
        "x":np.tile(np.arange(len(x_flat)),5),
        "y":np.concatenate([x_flat[:,0],x_flat[:,1],x_flat[:,2],x_flat[:,3],x_flat[:,4]]),
        "type":np.repeat(["cohesion","alignment","separation","steer_to_avoid","rotation"],len(x_flat))
        })
    px.line(features_df,x="x",y="y",color="type",title="Features").show()

