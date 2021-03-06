import numpy as np
import cmath
from tqdm import tqdm #might eventually make a param for this to ship, but later

class SimParams:
    def __init__(self,
    num_agents=10,
    dt=0.1,overall_time = 15,
    enclosure_size =10, init_pos_max= None,
    agent_max_vel=5,init_vel_max = None,agent_max_accel=np.inf,agent_max_turn_rate=np.inf,
    neighbor_radius=2,periodic_boundary=False):
        self.num_agents = num_agents
        self.dt = dt
        self.overall_time = overall_time
        self.enclosure_size = enclosure_size
        if init_pos_max is None:
            self.init_pos_max = enclosure_size
        else:
            self.init_pos_max = init_pos_max
        self.agent_max_vel = agent_max_vel
        if init_vel_max is None:
            self.init_vel_max = agent_max_vel
        else:
            self.init_vel_max = init_vel_max
        self.agent_max_accel = agent_max_accel
        self.agent_max_turn_rate = agent_max_turn_rate
        self.neighbor_radius = neighbor_radius
        self.periodic_boundary = periodic_boundary

def motionConstraints(vel_new,vel_last,params=SimParams()):
    maxAngleDeviation = params.agent_max_turn_rate*params.dt
    maxVelChange = params.agent_max_accel*params.dt

    #angle deviation of velocity
    if(np.linalg.norm(vel_new) > 0 and np.linalg.norm(vel_last) > 0):
        vel_new_angle = np.arctan2(vel_new[1],vel_new[0])
        vel_angle = np.arctan2(vel_last[1],vel_last[0])
        angleDeviation = vel_new_angle - vel_angle
        if angleDeviation > np.pi or angleDeviation < -np.pi:
            angleDeviation = -2*np.pi + angleDeviation

        if abs(angleDeviation) > maxAngleDeviation:
            agent_vel_hat = vel_last/np.linalg.norm(vel_last)
            agent_vel_complex = agent_vel_hat[0] + 1j*agent_vel_hat[1]
            agent_vel_complex *= np.linalg.norm(vel_new)
            # figure out which side to rotate
            if vel_angle + maxAngleDeviation > np.pi:
                maxAngleDeviation += 2*np.pi
            elif vel_angle - maxAngleDeviation < -1*np.pi:
                maxAngleDeviation -= 2*np.pi

            if angleDeviation > 0:
                agent_vel_complex *= cmath.exp(1j*-1*maxAngleDeviation)
            elif angleDeviation < 0:
                agent_vel_complex *= cmath.exp(1j*maxAngleDeviation)
            else:
                pass
            vel_new = np.array([agent_vel_complex.real,agent_vel_complex.imag])


    #max acceleration
    #have to account for going down to zero
    if np.linalg.norm(vel_new)-np.linalg.norm(vel_last) > maxVelChange:
        if np.linalg.norm(vel_new) == 0:
            vel_new = vel_last + maxVelChange*(vel_last/np.linalg.norm(vel_last))
        else:
            vel_new = vel_new*((maxVelChange+np.linalg.norm(vel_last))/np.linalg.norm(vel_new)) 
    elif np.linalg.norm(vel_new)-np.linalg.norm(vel_last) < -maxVelChange:
        if np.linalg.norm(vel_new) == 0:
            vel_new = vel_last - maxVelChange*(vel_last/np.linalg.norm(vel_last))
        else:
            vel_new = vel_new*((-maxVelChange+np.linalg.norm(vel_last))/np.linalg.norm(vel_new))

    #max vel
    if np.linalg.norm(vel_new) > params.agent_max_vel:
        vel_new *= (params.agent_max_vel/np.linalg.norm(vel_new))

    return vel_new

#note agentPositions and Vels at a single time
def neighborHood(pos,radius,agentPositions,agentVelocities):
    # fanciness might be unnessecary and slower, but it's short
    rel_pos = agentPositions - pos
    dist = np.linalg.norm(rel_pos,axis=1)
    inRadius = np.bitwise_and(dist <= radius, dist>0) #boolean array
   
    inRadius = np.repeat(inRadius,2) #make dimensions match with agentPositions and vels
    return np.extract(inRadius,agentPositions).reshape(-1,2),np.extract(inRadius,agentVelocities).reshape(-1,2)

#closed function to run whole sim and spit out vels and positions
def runSim(controllers=[],params=SimParams(),initial_positions=None,initial_velocities=None,progress_bar=False):
    steps = int(params.overall_time/params.dt)
    maxAngleDeviation = params.agent_max_turn_rate*params.dt
    maxVelChange = params.agent_max_accel*params.dt

    # define arrays of positions and vels over each step
    agentPositions = np.zeros([steps+1,params.num_agents,2])
    agentVels = np.zeros([steps+1,params.num_agents,2])

    #initial positions and velocities
    if initial_positions is None:
        for agentPos in agentPositions[0]:
            agentPos[0] = np.random.uniform(low=-params.init_pos_max,high=params.init_pos_max)
            agentPos[1] = np.random.uniform(low=-params.init_pos_max,high=params.init_pos_max)
    else:
        agentPositions[0] = initial_positions

    if initial_velocities is None:
        for agentVel in agentVels[0]:
            agentVel[0] = np.random.uniform(-params.init_vel_max,params.init_vel_max)
            agentVel[1] = np.random.uniform(-params.init_vel_max,params.init_vel_max)
    else:
        agentVels[0] = initial_velocities
    
    #run full simulation
    for step in (tqdm(range(0,steps)) if progress_bar else range(0,steps)):
        for agent in range(0,params.num_agents):
            agentPos = agentPositions[step,agent]
            agentVel = agentVels[step,agent]

            # next step is to abstract out BCs -- but not that important rn 
            if params.periodic_boundary == True:
                pass
            else:
                #Bounce Boundary Condition: invert last velocity and don't update
                if agentPos[0] > params.enclosure_size or agentPos[0] < -params.enclosure_size:
                    if((agentVel[0]<0 and agentPos[0]<-params.enclosure_size) or (agentVel[0]>0 and agentPos[0]>params.enclosure_size)):
                        agentVel[0] *= -1
                    agentVels[step+1,agent] = agentVel*(params.agent_max_vel/np.linalg.norm(agentVel))
                    agentPositions[step+1,agent] = agentPos  + agentVel*params.dt
                    continue

                if agentPos[1] > params.enclosure_size or agentPos[1] < -params.enclosure_size:
                    if((agentVel[1]<0 and agentPos[1]<-params.enclosure_size) or (agentVel[1]>0 and agentPos[1]>params.enclosure_size)):
                        agentVel[1] *= -1
                    agentVels[step+1,agent] = agentVel*(params.agent_max_vel/np.linalg.norm(agentVel))
                    agentPositions[step+1,agent] = agentPos  + agentVel*params.dt
                    continue

            #Periodic Boundary Condition - neighbor radius does not cross periodic boundaries
            if params.periodic_boundary == True:
                if agentPos[0] > params.enclosure_size or agentPos[0] < -params.enclosure_size:
                    if agentVel[0]<0 and agentPos[0]<-params.enclosure_size:
                        agentPos[0] += 2*params.enclosure_size
                    elif agentVel[0]>0 and agentPos[0]>params.enclosure_size:
                        agentPos[0] += -2*params.enclosure_size

                if agentPos[1] > params.enclosure_size or agentPos[1] < -params.enclosure_size:
                    if agentVel[1] < 0 and agentPos[1] < -params.enclosure_size:
                        agentPos[1] += 2 * params.enclosure_size
                    elif agentVel[1] > 0 and agentPos[1] > params.enclosure_size:
                        agentPos[1] += -2 * params.enclosure_size

            relevantPositions, relevantVels = neighborHood(agentPos,params.neighbor_radius,agentPositions[step],agentVels[step])

            vel_new = controllers[agent].vel(relevantPositions,relevantVels,agentPos,agentVel)

            #Implement agent motion constraints
            vel_new = motionConstraints(vel_new,agentVel,params=params)

            agentVels[step+1,agent] = vel_new
            agentPositions[step+1,agent] = vel_new*params.dt+agentPos
    return agentPositions, agentVels