#thoughts - get rid of inertia, have them follow last heading if no interaction
import numpy as np
from sim_tools.sim import SimParams,neighborHood
import sim_tools.media_export as export
from shapely import geometry
from multiprocessing import Pool
from functools import partial
from functools import reduce
from tqdm import tqdm

from dataclasses import dataclass



@dataclass
class posVelSlice:
    pos: np.ndarray
    vel: np.ndarray
    next_vel: np.ndarray

def toPosVelSlices(agentPositions,params=SimParams()):
    #includes velocity calculation--I don't fully understand how to abstract this out yet
    # would really like to remove this dt
    return [posVelSlice(agentPositions[i],
        (agentPositions[i]-agentPositions[i-1])/params.dt,
        (agentPositions[i+1]-agentPositions[i])/params.dt)
    for i in range(1,len(agentPositions)-1)]

@dataclass
class featureSlice:
    features: dict
    last_vel: np.ndarray
    output_vel: np.ndarray
    motion_constrained: bool
    boundary_constrained: bool

#rewriting this to generalized
def featuresFromPosVelSlice(slice,params=SimParams(),features={},neighborCaps=[0,np.inf]):
    featureSlices = []
    for agent in range(params.num_agents):
        agentPos = slice.pos[agent]
        agentVel = slice.vel[agent]
        agentNextVel = slice.next_vel[agent]

        over = agentPos - params.enclosure_size
        under = agentPos + params.enclosure_size
        
        # comparing both coordinates in a variety of ways
        # figure out how to set tolerances reasonably
        v_mag_current = np.linalg.norm(agentVel)
        v_mag_next = np.linalg.norm(agentNextVel)

        # 7 is the standard for 0.05 dt, just projecting linearly for other dts
        # yes this needs to be investigated further, but over rejection is ok for now
        stepsInFuturePast = 7 * (0.05/params.dt)

        tol = max(v_mag_current,v_mag_next)*params.dt*stepsInFuturePast
        above = over>=0
        closeTop = np.isclose(over,0,atol=tol)
        closeBot = np.isclose(under,0,atol=tol)
        below = under<=0

        boundary_constrained = False
        #elementiwise or of all the above boolean arrays
        if reduce(np.bitwise_or,[above,closeTop,below,closeBot]).any():
            boundary_constrained = True
        
        motion_constrained = False

        if np.linalg.norm(agentNextVel) >= params.agent_max_vel or np.isclose(np.linalg.norm(agentNextVel),params.agent_max_vel,atol=.1):
            motion_constrained = True
        # if np.linalg.norm(agentNextVel) > 6.7: print("Let",np.linalg.norm(agentNextVel),"get through")
        if abs(np.linalg.norm(agentNextVel)-np.linalg.norm(agentVel)) >= params.agent_max_accel*params.dt:
            motion_constrained = True

        vel_new_angle = np.arctan2(agentNextVel[1],agentNextVel[0])
        vel_angle = np.arctan2(agentVel[1],agentVel[0])
        angleDeviation = vel_new_angle - vel_angle
        if angleDeviation > np.pi or angleDeviation < -np.pi:
            angleDeviation = -2*np.pi + angleDeviation

        if abs(angleDeviation) >= params.agent_max_turn_rate*params.dt:
            motion_constrained = True
        
        relevantPositions,relevantVelocities = neighborHood(agentPos,params.neighbor_radius,slice.pos,slice.vel)

        # reject on neighbor counts
        if len(relevantPositions) < neighborCaps[0] or len(relevantPositions) > neighborCaps[1]:
            continue

        computed_features = {name:feature.compute(relevantPositions,relevantVelocities,agentPos,agentVel) for name,feature in features.items()}
        featureSlices.append(featureSlice(computed_features,agentVel,agentNextVel,motion_constrained,boundary_constrained))
    return featureSlices

def toFeatureSlices(posVelSlices,features={},params=SimParams(),threads = 8,neighborCaps=[1,np.inf],verbose=True):
    #multiprocessing
    pool = Pool(threads)
    
    #partial eval to pass in params, normal function currying doesn't work
    clumpedSlices = []
    if verbose:
        clumpedSlices = tqdm(pool.imap(
            partial(featuresFromPosVelSlice,features=features,params=params,neighborCaps=neighborCaps),
            posVelSlices),total=len(posVelSlices))
    else:
        clumpedSlices = pool.map(
            partial(featuresFromPosVelSlice,features=features,params=params,neighborCaps=neighborCaps),
            posVelSlices)

    featuresSlices = []
    for clump in clumpedSlices:
        featuresSlices.extend(clump)

    if verbose:
        print("Ignored ",(len(posVelSlices)*params.num_agents)-len(featuresSlices),"/",len(posVelSlices)*params.num_agents," slices")
    return featuresSlices



@dataclass
class agentSlice:
    #inputs
    cohesion: np.ndarray
    alignment: np.ndarray
    separation: np.ndarray
    steer_to_avoid:np.ndarray
    rotation: np.ndarray
    last_vel: np.ndarray
    #output
    output_vel: np.ndarray

#need to figure out how to pass more things to this, maybe function currying is the way
def agentSliceFromPosVelSlice(slice,params=SimParams(),neighborCaps=[0,np.inf],ignoreConstrainedMotion=False,ignoreBoundaryData=True):
    # if ignoreConstrainedMotion: print("IGNORING CONSTRAINED MOTION")
    agentSlices = []
    for agent in range(params.num_agents):
        #calculate all relevant derivate metrics, repackage
        posCentroid = np.zeros(2)
        velCentroid = np.zeros(2)
        agentPos = slice.pos[agent]

        agentVel = slice.vel[agent]
        agentNextVel = slice.next_vel[agent] #- agentVel
        separation = np.zeros(2)

        if ignoreBoundaryData:
            #throw out data near the boundary, only needed with bounce
            if(agentPos[0] > params.enclosure_size or agentPos[0] < -params.enclosure_size 
            or agentPos[1] > params.enclosure_size or agentPos[1] < -params.enclosure_size):
                continue


        
        if ignoreConstrainedMotion:
            if np.linalg.norm(agentNextVel) >= params.agent_max_vel or np.isclose(np.linalg.norm(agentNextVel),params.agent_max_vel,atol=.1):
                # print("ignoring at max vel")
                continue
            # if np.linalg.norm(agentNextVel) > 6.7: print("Let",np.linalg.norm(agentNextVel),"get through")
            if abs(np.linalg.norm(agentNextVel)-np.linalg.norm(agentVel)) >= params.agent_max_accel*params.dt:
                continue
    
            vel_new_angle = np.arctan2(agentNextVel[1],agentNextVel[0])
            vel_angle = np.arctan2(agentVel[1],agentVel[0])
            angleDeviation = vel_new_angle - vel_angle
            if angleDeviation > np.pi or angleDeviation < -np.pi:
                angleDeviation = -2*np.pi + angleDeviation
    
            if abs(angleDeviation) >= params.agent_max_turn_rate*params.dt:
                continue
        

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
        
        #agents move sideways relative to centroid
        relative_pos = np.zeros(2)
        relative_pos = posCentroid-agentPos
        k = relative_pos[1]
        relative_pos[1] = relative_pos[0]
        relative_pos[0] = -k
        rotation = relative_pos/np.linalg.norm(relative_pos)

        #throw out data without interactions
        if adjacent < neighborCaps[0] or adjacent > neighborCaps[1]:
            continue

        posCentroid /= adjacent
        velCentroid /= adjacent

        #steer-to-avoid
        origin = geometry.Point(agentPos[0],agentPos[1])

        # need to pass params for neighborhood stuff
        orientation = (agentVel/np.linalg.norm(agentVel))*params.neighbor_radius
        toward = geometry.Point(agentPos[0]+orientation[0],agentPos[1]+orientation[1])        

        velLine = geometry.LineString([origin,toward])

        steerToAvoid = np.zeros(2)
        # this can be considered a property of the environment?, how is agent's size defined
        collision_distance = params.neighbor_radius/4

        closest = np.zeros(2)
        closestDist = np.inf
        for otherAgent in range(params.num_agents):
            position = slice.pos[otherAgent]
            other = geometry.Point(position[0],position[1])
            collision = other.buffer(collision_distance)
            if velLine.intersects(collision):
                dist = np.linalg.norm(position-agentPos)
                if(dist < closestDist and closestDist != 0):
                    closestDist = dist
                    closest = position
        
        #assign 
        if closestDist != np.inf and closestDist != 0:
            mag = 1/(closestDist**2)
            #figure out side
            diffPos = closest-agentPos
            #project onto v, grab remaining component as orthogonal direction
            v_hat = agentVel/np.linalg.norm(agentVel)
            d_v = np.dot(v_hat,diffPos)*v_hat
            remaining = diffPos - d_v
            if np.linalg.norm(remaining) > 0:
                remaining = remaining/np.linalg.norm(remaining)
            # print("Verify orthogonal: ",np.dot(remaining,v))
            # print("DiffPos",diffPos)
            # print("D_v",d_v)
            # print("D_r",remaining)
            # print("Sign",np.sign(np.cross(remaining,v)))
            steerToAvoid = remaining*mag
        agentSlices.append(agentSlice(posCentroid-agentPos,velCentroid,separation,steerToAvoid,rotation,agentVel,agentNextVel))
    return agentSlices

#this can be multiprocessed, maybe also allow turning off certain slices
def toAgentSlices(posVelSlices,params=SimParams(),threads = 8,neighborCaps=[1,np.inf],ignoreConstrainedMotion=False,ignoreBoundaryData=True,verbose=True):
    #multiprocessing
    pool = Pool(threads)
    
    #partial eval to pass in params, normal function currying doesn't work
    clumpedSlices = []
    if verbose:
        clumpedSlices = tqdm(pool.imap(
            partial(agentSliceFromPosVelSlice,params=params,neighborCaps=neighborCaps,ignoreConstrainedMotion=ignoreConstrainedMotion,ignoreBoundaryData=ignoreBoundaryData),
            posVelSlices),total=len(posVelSlices))
    else:
        clumpedSlices = pool.map(
            partial(agentSliceFromPosVelSlice,params=params,neighborCaps=neighborCaps,ignoreConstrainedMotion=ignoreConstrainedMotion,
            ignoreBoundaryData=ignoreBoundaryData)
            ,posVelSlices)

    agentSlices = []
    for clump in clumpedSlices:
        agentSlices.extend(clump)

    if verbose:
        print("Ignored ",(len(posVelSlices)*params.num_agents)-len(agentSlices),"/",len(posVelSlices)*params.num_agents," slices")
    return agentSlices

