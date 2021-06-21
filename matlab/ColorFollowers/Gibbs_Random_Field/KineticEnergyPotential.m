function KEPotential = KineticEnergyPotential(centralVelocity, centralClass, neighborVelocities, neighborClasses, agentMass)
    
neighborsRelativeVelocities = neighborVelocities - centralVelocity;
groupVelocity = sum(neighborsRelativeVelocities(neighborClasses==centralClass),1);
groupMass = agentMass * size(neighborsRelativeVelocities(neighborClasses==centralClass),1);
KEPotential = 0.5*groupMass * dot(groupVelocity,groupVelocity);

end