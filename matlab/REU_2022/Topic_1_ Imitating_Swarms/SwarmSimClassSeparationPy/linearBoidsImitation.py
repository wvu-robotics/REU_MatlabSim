import numpy as np
import sim
import media_export as export


from sklearn.linear_model import LinearRegression as lr
from tqdm import tqdm
from dataclasses import dataclass #data class is like the python equivalent of a struct
from models import Boids as bo

params = sim.SimParams(
    num_agents=30, 
    dt=0.05, 
    overall_time = 15, 
    enclosure_size = 10, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=5,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=3,
    periodic_boundary=False
    )

#constants to imitate
k_coh = 3
k_align = 5
k_sep = .01
k_inertia = 0

true_gains = [k_coh, k_align, k_sep, k_inertia]

#run sim
# print("Original agent slices")

#ran first sim
controllers = [bo.Boids(*true_gains) for i in range(params.num_agents)]
print("First sim and export")
agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

export.export(export.ExportType.GIF,"Initial",agentPositions,params=params,vision_mode=False,progress_bar=True)



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
print("Running short sims")
params.overall_time=3
extra_sims = 100
for extra_sim in tqdm(range(extra_sims)):
    agentPositions, agentVels = sim.runSim(controllers,params)
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

agentSlices = []
for slice in posVelSlices:
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

        # throw out data that is at the edge of action constraints
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

# for slice in agentSlices:
#     print(slice)

#do linear regression on these things, and get the coefficients, do some train/test split

#another reformatting to be in x,y format for linear regression

#currently getting rid of half the data, which is bad
x = []
y = []

#reshapes were being annoying, will rewrite better
for slice in agentSlices:
    x.append(np.array([slice.cohesion[0],slice.alignment[0],slice.separation[0],slice.last_vel[0]]))
    y.append(np.array(slice.output_vel[0]))
    x.append(np.array([slice.cohesion[1],slice.alignment[1],slice.separation[1],slice.last_vel[1]]))
    y.append(np.array(slice.output_vel[1]))

# print("True loss: ",true_loss)
reg = lr().fit(x,y)
print("R^2: ",reg.score(x,y))

# #create some visuals with the imitated swarm
#     #maybe do an imposter(s) pretending to be within the original swarm
gains = reg.coef_.tolist()
gains[3]=k_inertia

print(gains)

controllers_imitated = [bo.Boids(*gains) for i in range(params.num_agents)]

params.overall_time = 15
#start at exactly the same place
print("Running final visual")
agentPositions_imitated, agentVels_imitated = sim.runSim(controllers_imitated,params,initial_positions=agentPositions[0],initial_velocities=agentVels[0],progress_bar=True)
export.export(export.ExportType.GIF,"Imitated",agentPositions_imitated,params=params,vision_mode=False,progress_bar=True)
