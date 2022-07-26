from operator import ne
import numpy as np
import shapely.geometry as geometry
import shapely.ops as ops

#class wrapper for generic non linear feature
# I should put acceptable bounds with each features
class Feature:
    # outputs 2d vel part nonlinearly, just from relevant positions and vels(only ones passed)
    def compute(self,agentPositions,agentVels,pos=np.zeros(2),v=np.zeros(2)):
        pass

#input every new feature here, WRITE AS CONCISE AS POSSIBLE, use numpy and others

class Cohesion(Feature):
    def compute(self,agentPositions, agentVels, pos, v):
        centroid = np.mean(agentPositions,axis=0)
        return centroid - pos

class Alignment(Feature):
    def compute(self,agentPositions, agentVels, pos, v):
        return np.mean(agentVels,axis=0)

# might make a polynomial separation factory later
class SeparationInv2(Feature):
    def compute(self,agentPositions, agentVels, pos, v):
        rel_pos = agentPositions - pos
        dist_weighted = np.array([-1*(1/(np.linalg.norm(pos)**2)) for pos in rel_pos])
        unit_rel_pos = np.array([pos/np.linalg.norm(pos) for pos in rel_pos])
        combined_pos = (dist_weighted*unit_rel_pos.transpose()).transpose()
        return np.sum(combined_pos,axis=0)

class SeparationInv6(Feature):
    def compute(self,agentPositions, agentVels, pos, v):
        rel_pos = agentPositions - pos
        dist_weighted = np.array([-1*(1/(np.linalg.norm(pos)**6)) for pos in rel_pos])
        unit_rel_pos = np.array([pos/np.linalg.norm(pos) for pos in rel_pos])
        combined_pos = (dist_weighted*unit_rel_pos.transpose()).transpose()
        return np.sum(combined_pos,axis=0)

class SeparationInv12(Feature):
    def compute(self,agentPositions, agentVels, pos, v):
        rel_pos = agentPositions - pos
        dist_weighted = np.array([-1*(1/(np.linalg.norm(pos)**12)) for pos in rel_pos])
        unit_rel_pos = np.array([pos/np.linalg.norm(pos) for pos in rel_pos])
        combined_pos = (dist_weighted*unit_rel_pos.transpose()).transpose()
        return np.sum(combined_pos,axis=0)


class SeparationExp(Feature):
    def compute(self, agentPositions, agentVels, pos, v):
        rel_pos = agentPositions - pos
        dist_weighted = np.array([-1*(1/(np.exp(np.linalg.norm(pos))-1)) for pos in rel_pos])
        unit_rel_pos = np.array([pos/np.linalg.norm(pos) for pos in rel_pos])
        combined_pos = (dist_weighted*unit_rel_pos.transpose()).transpose()
        return np.sum(combined_pos,axis=0)

#TODO exponential separation/cohesion together

# could probably be shortened a lot, but not rewriting atm
class SteerToAvoid(Feature):
    def __init__(self,collision_radius,neighbor_radius):
        self.collision_radius = collision_radius
        self.neighbor_radius = neighbor_radius

    def compute(self,agentPositions, agentVels, pos, v):
        origin = geometry.Point(pos[0],pos[1])
        if np.linalg.norm(v) ==  0:
            return np.zeros(2)
        # print(v)
        orientation = (v/np.linalg.norm(v))*self.neighbor_radius
        toward = geometry.Point(pos[0]+orientation[0],pos[1]+orientation[1])

        velLine = geometry.LineString([origin,toward])

        closest = np.zeros(2)
        closestDist = np.inf

        for position in agentPositions:
            other = geometry.Point(position[0],position[1])
            collision = other.buffer(self.collision_radius)
            if velLine.intersects(collision):
                dist = np.linalg.norm(position-pos)
                if(dist < closestDist):
                    closestDist = dist
                    closest = position

        steerToAvoid = np.zeros(2)
        #assign steerToAvoid
        if closestDist != np.inf:
            mag = 1/(closestDist**2)
            #figure out side
            diffPos = closest-pos
            #project onto v, grab remaining component as orthogonal direction
            v_hat = v/np.linalg.norm(v)
            d_v = np.dot(v_hat,diffPos)*v_hat
            remaining = diffPos - d_v
            if np.linalg.norm(remaining) > 0:
                remaining = remaining/np.linalg.norm(remaining)
            steerToAvoid = remaining*mag
        return steerToAvoid

class Rotation(Feature):
    def compute(self,agentPositions, agentVels, pos, v):
        centroid = np.mean(agentPositions,axis=0)
        toCentroid =  centroid - pos
        return np.array([-toCentroid[1],toCentroid[0]])

class Inertia(Feature):
    def compute(self,agentPositions, agentVels, pos, v):
        return v

"""
class RectangularBoundSeparation(Feature):
    def __init__(self, xEnclosureSize, yEnclosureSize, neighbor_radius):
        self.xEnclosureSize = xEnclosureSize
        self.yEnclosureSize = yEnclosureSize
        self.neighbor_radius = neighbor_radius
    def compute(self, agentPositions, agentVelts, pos, v):
        rel_pos_to_max = np.array([self.xEnclosureSize-pos[0],self.yEnclosureSize-pos[1]])
        rel_pos_to_min = np.array([-self.xEnclosureSize-pos[0],-self.yEnclosureSize-pos[1]])
        impulse = np.array([0,0])
        if pos[0] + self.neighbor_radius > self.xEnclosureSize:
            impulse[0] = -1/(rel_pos_to_max[0]**2)
        if pos[1] + self.neighbor_radius > self.yEnclosureSize:
            impulse[1] = -1/(rel_pos_to_max[1]**2)
        if pos[0] - self.neighbor_radius < -self.xEnclosureSize:
            impulse[0] = 1/((rel_pos_to_min[0]**2))
        if pos[1] - self.neighbor_radius < -self.yEnclosureSize:
            impulse[1] = 1/((rel_pos_to_min[1]**2))
        return impulse
        """

class RectangularBoundSeparationShapely(Feature):
    def __init__(self, xEnclosureSize, yEnclosureSize, neighbor_radius):
        self.xEnclosureSize = xEnclosureSize
        self.yEnclosureSize = yEnclosureSize
        self.neighbor_radius = neighbor_radius
    def compute(self, agentPositions, agentVelts, pos, v):
        arena = geometry.LinearRing([
            (-self.xEnclosureSize, -self.yEnclosureSize),
            (-self.xEnclosureSize, self.yEnclosureSize),
            (self.xEnclosureSize, self.yEnclosureSize),
            (self.xEnclosureSize,-self.yEnclosureSize)
        ])
        agentpoint = geometry.Point(pos[0], pos[1])
        polypoint, pointpoint = ops.nearest_points(arena, agentpoint)
        dist_weighted = np.array([0,0])
        unit_dist = np.array([0,0])
        rel_pos = np.array([polypoint.x - pos[0], polypoint.y - pos[1]])
        if agentpoint.distance(arena) <= self.neighbor_radius:
            dist_weighted = -1/(agentpoint.distance(arena))**2
            unit_dist = rel_pos/agentpoint.distance(arena)
        impulse = dist_weighted*unit_dist

        return impulse

class RectangularBoundSteer(Feature):
    def __init__(self, xEnclosureSize, yEnclosureSize, neighbor_radius):
        self.xEnclosureSize = xEnclosureSize
        self.yEnclosureSize = yEnclosureSize
        self.neighbor_radius = neighbor_radius
    def compute(self, agentPositions, agentVelts, pos, v):
        arena = geometry.LinearRing([
            (-self.xEnclosureSize, -self.yEnclosureSize),
            (-self.xEnclosureSize, self.yEnclosureSize),
            (self.xEnclosureSize, self.yEnclosureSize),
            (self.xEnclosureSize,-self.yEnclosureSize)
        ])
        agentpoint = geometry.Point(pos[0], pos[1])
        polypoint, pointpoint = ops.nearest_points(arena, agentpoint)
        rel_pos = np.array([polypoint.x - pos[0], polypoint.y - pos[1]])
        impulse = np.array([0,0])
        if agentpoint.distance(arena) <= self.neighbor_radius:
            cross = np.cross(rel_pos,v)
            dot = np.dot(rel_pos,v)
            if dot > 0:
                if cross > 0:
                    impulse = np.array([-polypoint.y + rel_pos[1], polypoint.x - rel_pos[0]])
                if cross <= 0:
                    impulse = -1*np.array([-polypoint.y + rel_pos[1], polypoint.x - rel_pos[0]])
        return impulse

    







        



