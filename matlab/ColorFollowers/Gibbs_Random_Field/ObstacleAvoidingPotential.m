function OAPotential = ObstacleAvoidingPotential(centralPosition, possibleVelocity, centralCharge, obstacleLocations, obstacleCharge, sensingRange, epsilon, epsilon_not, alpha, r_not, timeStep)
    
    distToObstacles = obstacleLocation - centralPosition;
    closeObstacles = obstacleLocations(distToObstacles <= sensingRange);
    
    newPos = KinematicModel(centralPosition, possibleVelocity, timeStep);
    
    OAPotential = 0;
    for i = 1:size(closeObstacles, 1)
        neighborClass = centralClass;
       CBPotential = CoulombBuckingham(newPos, closeObstacles(i,:), centralClass, ...
           neighborClass, centralCharge, obstacleCharge, epsilon, epsilon_not, alpha, r_not);
        OAPotential = OAPotential + CBPotential;
    end
end