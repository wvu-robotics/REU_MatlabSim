# import sys
# sys.path.append("..")
import numpy as np
import sim_tools.sim as sim
import sim_tools.media_export as export
import copy
import os 

import imitation_tools.automation as automation

from sklearn.linear_model import LinearRegression as lr
from tqdm import tqdm
from dataclasses import dataclass #data class is like the python equivalent of a struct
from models import Boids as bo
from models import Dance

params = sim.SimParams(
    num_agents=50,
    dt=0.01,
    overall_time = 10,
    enclosure_size = 20,
    init_pos_max = None, #if None, then defaults to enclosure_size
    agent_max_vel=10,
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
    #constants to imitate
    k_coh = 10
    k_align = 1
    k_sep = 1
    k_inertia = 1

    true_gains = [k_coh, k_align, k_sep, k_inertia]

    #run sim
    # print("Original agent slices")

    #ran first sim
    controllers = [bo.Boids(*true_gains) for i in range(params.num_agents)]
    print("First sim and export")
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

    if not os.path.exists("linearBoidsOutput"):
        os.makedirs("linearBoidsOutput")

    export.export(export.ExportType.MP4,"linearBoidsOutput/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)



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
    shortControllers = [bo.Boids(*true_gains) for i in range(shortSimParams.num_agents)]
    #colors = ["rgb(255, 0, 24)","rgb(255, 165, 44)","rgb(255, 255, 65)","rgb(0, 128, 24)","rgb(0, 0, 249)","rgb(134, 0, 125)","rgb(85, 205, 252)","rgb(247, 168, 184)"]

    agentSlices = automation.runSims(shortControllers,params=shortSimParams,num_sims=100,ignoreMC=True,
    export_info=[export.ExportType.MP4,"linearBoidsOutput/ShortSims",100])
    print("Num agent slices",len(agentSlices))

    x = []
    y = []

    #reshapes were being annoying, will rewrite better
    for slice in agentSlices:
        x.append(np.array([slice.cohesion[0],slice.alignment[0],slice.separation[0]]))
        y.append(np.array(slice.output_vel[0]))
        x.append(np.array([slice.cohesion[1],slice.alignment[1],slice.separation[1]]))
        y.append(np.array(slice.output_vel[1]))

    # print("True loss: ",true_loss)
    reg = lr().fit(x,y)
    print("R^2: ",reg.score(x,y))

    # #create some visuals with the imitated swarm
    #     #maybe do an imposter(s) pretending to be within the original swarm
    gains = reg.coef_.tolist()
    gains.append(k_inertia)

    print(gains)

    controllers_imitated = [bo.Boids(*gains) for i in range(params.num_agents)]
    for controller in controllers_imitated:
        controller.setColor("red")

    #start at exactly the same place
    print("Running final visual")
    agentPositions_imitated, agentVels_imitated = sim.runSim(controllers_imitated,params,initial_positions=agentPositions[0],initial_velocities=agentVels[0],progress_bar=True)
    export.export(export.ExportType.MP4,"linearBoidsOutput/Imitated",agentPositions_imitated,agentVels_imitated,controllers=controllers_imitated,params=params,vision_mode=False,progress_bar=True)


    # now create some hybrid visualizations
    print("Running hybrid visual")

    #some parameters for hybrid visualization
    mix_factor = 0.5
    params.num_agents = 100
    params.enclosure_size = 20
    params.overall_time = 20
    params.init_pos_max = params.enclosure_size
    params.agent_max_vel = 10

    original_agents = [bo.Boids(*true_gains) for i in range(int(params.num_agents*mix_factor))]
    for controller in original_agents:
        controller.setColor("rgb(99, 110, 250)")
    imitated_agents = [bo.Boids(*gains) for i in range(int(params.num_agents*(1-mix_factor)))]
    for controller in imitated_agents:
        controller.setColor("red")

    all_controllers = original_agents + imitated_agents

    agentPositions_hybrid, agentVels_hybrid = sim.runSim(all_controllers,params,progress_bar=True)
    export.export(export.ExportType.MP4,"linearBoidsOutput/Hybrid",agentPositions_hybrid,agentVels_hybrid,controllers=all_controllers,params=params,vision_mode=False,progress_bar=True)



    """
    original_agents = [bo.Boids(*true_gains) for i in range(int(params.num_agents*mix_factor))]
    for controller in original_agents:
        controller.setColor("rgb(99, 110, 250)")
    imitated_agents = [bo.Boids(*gains) for i in range(int(params.num_agents*(1-mix_factor)))]
    for controller in imitated_agents:
        controller.setColor("black")

    all_controllers = original_agents + imitated_agents

    agentPositions_hybrid, agentVels_hybrid = sim.runSim(all_controllers,params,progress_bar=True,initial_positions=agentPositions_hybrid[-1],initial_velocities=agentVels_hybrid[-1])
    export.export(export.ExportType.MP4,"linearBoidsOutput/HybridNonuniform",agentPositions_hybrid,agentVels_hybrid,controllers=all_controllers,params=params,vision_mode=False,progress_bar=True)
    """

