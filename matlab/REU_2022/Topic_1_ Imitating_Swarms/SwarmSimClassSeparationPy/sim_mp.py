from re import S
import numpy as np
import cmath
from tqdm import tqdm #might eventually make a param for this to ship, but later
from multiprocessing import Pool

class SimParams:
    def __init__(self,
    num_agents=10,
    dt=0.1,overall_time = 15,
    enclosure_size =10, init_pos_max= None,
    agent_max_vel=5,agent_max_accel=1,agent_max_turn_rate=np.inf,
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
        self.agent_max_accel = agent_max_accel
        self.agent_max_turn_rate = agent_max_turn_rate
        self.neighbor_radius = neighbor_radius
        self.periodic_boundary = periodic_boundary

#used for multiprocessing pooling
def updateSubSlice(indices,controllers,posSlice,velSlice,nextPos,nextVels,params=SimParams()):
    #indexing is a bit funny,nextPos, nextVels are indexed 0-n
    #posSlice,velSlice, controllers are indexed globally, and indices is the mapping
    maxAngleDeviation = params.agent_max_turn_rate*params.dt
    maxVelChange = params.agent_max_accel*params.dt

    # print("All agents: ",indices)

    for i in range(len(controllers)):
        # print("Agent:",indices[i])
        agent = indices[i]
        agentPos = posSlice[agent]
        agentVel = velSlice[agent]

        if params.periodic_boundary == True:
            pass
        else:
            #Bounce Boundary Condition: invert last velocity and don't update
            if agentPos[0] > params.enclosure_size or agentPos[0] < -params.enclosure_size:
                if((agentVel[0]<0 and agentPos[0]<-params.enclosure_size) or (agentVel[0]>0 and agentPos[0]>params.enclosure_size)):
                    agentVel[0] *= -1
                nextVels[i] = agentVel*(params.agent_max_vel/np.linalg.norm(agentVel))
                nextPos[i] = agentPos  + agentVel*params.dt
                continue

            if agentPos[1] > params.enclosure_size or agentPos[1] < -params.enclosure_size:
                if((agentVel[1]<0 and agentPos[1]<-params.enclosure_size) or (agentVel[1]>0 and agentPos[1]>params.enclosure_size)):
                    agentVel[1] *= -1
                nextVels[i] = agentVel*(params.agent_max_vel/np.linalg.norm(agentVel))
                nextPos[i] = agentPos  + agentVel*params.dt
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


        relevantPositions = []
        relevantVels = []

        for other_agent in range(0,params.num_agents):
            if agent == other_agent or np.linalg.norm(agentPos-posSlice[other_agent]) > params.neighbor_radius:
                continue
            relevantPositions.append(posSlice[other_agent])
            relevantVels.append(velSlice[other_agent])

        vel_new = controllers[agent].vel(relevantPositions,relevantVels,agentPos,agentVel)

        #Constraints

        #angle
        if(np.linalg.norm(vel_new) > 0 and np.linalg.norm(agentVel) > 0):
            vel_new_angle = np.arctan2(vel_new[1],vel_new[0])
            vel_angle = np.arctan2(agentVel[1],agentVel[0])
            angleDeviation = vel_new_angle - vel_angle
            if angleDeviation > np.pi or angleDeviation < -np.pi:
                angleDeviation = -2*np.pi + angleDeviation

            if abs(angleDeviation) > maxAngleDeviation:
                agent_vel_hat = agentVel/np.linalg.norm(agentVel)
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
        if np.linalg.norm(vel_new)-np.linalg.norm(agentVel) > maxVelChange:
            if np.linalg.norm(vel_new) == 0:
                vel_new = agentVel + maxVelChange*(agentVel/np.linalg.norm(agentVel))
            else:
                vel_new = vel_new*((maxVelChange+np.linalg.norm(agentVel))/np.linalg.norm(vel_new)) 
        elif np.linalg.norm(vel_new)-np.linalg.norm(agentVel) < -maxVelChange:
            if np.linalg.norm(vel_new) == 0:
                vel_new = agentVel - maxVelChange*(agentVel/np.linalg.norm(agentVel))
            else:
                vel_new = vel_new*((-maxVelChange+np.linalg.norm(agentVel))/np.linalg.norm(vel_new))

        #max vel
        if np.linalg.norm(vel_new) > params.agent_max_vel:
            vel_new *= (params.agent_max_vel/np.linalg.norm(vel_new))
        
        #ISSUES WITH WRITING
        print(vel_new)
        nextVels[i] = vel_new
        nextPos[i] = agentPos  + agentVel*params.dt

#closed function to run whole sim and spit out vels and positions
def runSim(controllers=[],params=SimParams(),initial_positions=None,initial_velocities=None,pools=4,progress_bar=False):
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
            agentVel[0] = np.random.uniform(-params.agent_max_vel,params.agent_max_vel)
            agentVel[1] = np.random.uniform(-params.agent_max_vel,params.agent_max_vel)
    else:
        agentVels[0] = initial_velocities
    
    #run full simulation
    for step in (tqdm(range(0,steps)) if progress_bar else range(0,steps)):
        print("Step: ",step)
        # print("Positions: ",agentPositions[step])
        # print("Vels: ",agentVels[step])
        pool = Pool(processes=pools)
        #run slices asyncrhonously
        # currently set up to presplit the info and let them go, could also just do a map and not need to batch things
        batchSize = int(params.num_agents/pools)
        for i in range(pools+1):
            start = i*batchSize
            end = min((i+1)*batchSize,params.num_agents)
            pool.apply_async(updateSubSlice,
                args=(range(start,end),controllers,agentPositions[step],agentVels[step],
                    agentPositions[step,start:end],agentVels[step,start:end],params))

        # pool.map(updateSubSlice,range(0,params.num_agents))
        pool.close() # might be better to reuse pool, but it's hard
        pool.join()
    return agentPositions, agentVels