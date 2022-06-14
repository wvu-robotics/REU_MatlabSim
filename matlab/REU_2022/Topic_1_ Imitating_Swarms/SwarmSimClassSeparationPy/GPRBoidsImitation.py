import numpy as np
import sim
import media_export as export
import copy

from sklearn.linear_model import LinearRegression as lr
from tqdm import tqdm
from dataclasses import dataclass #data class is like the python equivalent of a struct
from models import Boids as bo

from sklearn.gaussian_process import GaussianProcessRegressor as gp
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C

params = sim.SimParams(
    num_agents=40, 
    dt=0.05, 
    overall_time = 15, 
    enclosure_size = 10, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=7,
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


#constants to imitate
k_coh = 3
k_align = 5
k_sep = 1
k_inertia = 1

true_gains = [k_coh, k_align, k_sep, k_inertia]

#run sim
# print("Original agent slices")

#ran first sim
controllers = [bo.Boids(*true_gains) for i in range(params.num_agents)]
print("First sim and export")
agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

export.export(export.ExportType.GIF,"Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)



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
shortSimParams.overall_time = 2
shortSimParams.enclosure_size = 2*params.enclosure_size
shortSimParams.init_pos_max = params.enclosure_size/2

extra_sims = 100
for extra_sim in tqdm(range(extra_sims)):
    agentPositions, agentVels = sim.runSim(controllers,shortSimParams)
    posVelSlices.extend([posVelSlice(agentPositions[i],agentVels[i],agentVels[i+1]) for i in range(len(agentPositions)-1)])

# print("PosVelSlices:")
# print(posVelSlices)

#reformat slices to be relevant by agent, getting the centroid, separation, alignment, and last vel
@dataclass
class agentSlice:
    #inputs
    cohesion: np.ndarray
    alignment: np.ndarray
    separation: np.ndarray
    last_vel: np.ndarray
    #output
    output_vel: np.ndarray

print("Parsing slices")
agentSlices = []
for slice in tqdm(posVelSlices):
    for agent in range(params.num_agents):
        #calculate all relevant derivate metrics, repackage
        posCentroid = np.zeros(2)
        velCentroid = np.zeros(2)
        agentPos = slice.pos[agent]
    
        #throw out data near the boundary, only needed with bounce
        if(agentPos[0] > params.enclosure_size or agentPos[0] < -params.enclosure_size 
        or agentPos[1] > params.enclosure_size or agentPos[1] < -params.enclosure_size):
            continue

        agentVel = slice.vel[agent]
        agentNextVel = slice.next_vel[agent]
        separation = np.zeros(2)
        agentVelDifference = agentNextVel #- agentVel

        # throw out data that is at the edge of action constraints--might add some probability
        if np.linalg.norm(agentVel) >= params.agent_max_vel:
            continue
        
        # if np.linalg.norm(agentVelDifference) >= params.agent_max_accel:
        #     continue
        
        # vel_new_angle = np.arctan2(agentNextVel[1],agentNextVel[0])
        # vel_angle = np.arctan2(agentVel[1],agentVel[0])
        # angleDeviation = vel_new_angle - vel_angle
        # if angleDeviation > np.pi or angleDeviation < -np.pi:
        #     angleDeviation = -2*np.pi + angleDeviation
        
        # if abs(angleDeviation) >= params.agent_max_turn_rate*params.dt:
        #     continue

        adjacent = 0
        for otherAgent in range(params.num_agents):
            if otherAgent == agent:
                continue
            
            otherPos = slice.pos[otherAgent]

            if np.linalg.norm(otherPos-agentPos) > params.neighbor_radius:
                continue
            
            dist = np.linalg.norm(otherPos-agentPos)

            separation += ((otherPos-agentPos)/dist)*-1*(1/(dist**2))
            otherVel = slice.vel[otherAgent]
            posCentroid += otherPos
            velCentroid += otherVel
            adjacent += 1
        
        #throw out data without interactions
        if adjacent < 1:
            continue

        posCentroid /= adjacent
        velCentroid /= adjacent



        agentSlices.append(agentSlice(posCentroid-agentPos,velCentroid,separation,np.zeros(2),agentVelDifference))

print("Ignored ",(len(posVelSlices)*params.num_agents)-len(agentSlices),"/",len(posVelSlices)*params.num_agents," slices")

x = []
y = []

#reshapes were being annoying, will rewrite better
for slice in agentSlices:
    x.append(np.array([slice.cohesion[0],slice.alignment[0],slice.separation[0],slice.last_vel[0]]))
    y.append(np.array(slice.output_vel[0]))
    x.append(np.array([slice.cohesion[1],slice.alignment[1],slice.separation[1],slice.last_vel[1]]))
    y.append(np.array(slice.output_vel[1]))

# print("True loss: ",true_loss)

reg = gp().fit(x,y)
print("R^2: ",reg.score(x,y))

# #create some visuals with the imitated swarm
#     #maybe do an imposter(s) pretending to be within the original swarm
"""gains = reg.coef_.tolist()
gains[3]=k_inertia

print(gains)

controllers_imitated = [bo.Boids(*gains) for i in range(params.num_agents)]
for controller in controllers_imitated:
    controller.setColor("black")

#start at exactly the same place
print("Running final visual")
agentPositions_imitated, agentVels_imitated = sim.runSim(controllers_imitated,params,initial_positions=agentPositions[0],initial_velocities=agentVels[0],progress_bar=True)
export.export(export.ExportType.GIF,"Imitated",agentPositions_imitated,controllers=controllers_imitated,params=params,vision_mode=False,progress_bar=True)


# now create some hybrid visualizations
print("Running hybrid visual")
mix_factor = 0.6
original_agents = [bo.Boids(*true_gains) for i in range(int(params.num_agents*mix_factor))]
for controller in original_agents:
    controller.setColor("rgb(99, 110, 250)")
imitated_agents = [bo.Boids(*gains) for i in range(int(params.num_agents*(1-mix_factor)))]
for controller in imitated_agents:
    controller.setColor("black")

all_controllers = original_agents + imitated_agents

agentPositions_hybrid, agentVels_hybrid = sim.runSim(all_controllers,params,initial_positions=agentPositions[0],initial_velocities=agentVels[0],progress_bar=True)
export.export(export.ExportType.GIF,"Hybrid",agentPositions_hybrid,controllers=all_controllers,params=params,vision_mode=False,progress_bar=True)
"""