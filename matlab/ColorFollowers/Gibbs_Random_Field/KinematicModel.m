function newPos = KinematicModel(position, velocity, timeStep)


newPos = position + velocity * timeStep;


end