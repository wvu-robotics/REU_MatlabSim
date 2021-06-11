function [newVelocities, numCollisions] = Collider(agentPositions, velocityControls, agentRadius, timeStep)

    numCollisions = 0;
for i = 1:size(agentPositions,1)
   
    centralAgentPos = agentPositions(i,:);
    centralAgentVelControl = velocityControls(i,:);
    centralAgentNewPosition = centralAgentPos + timeStep * centralAgentVelControl;
    relNeighborPositions = agentPositions;
    relNeighborPositions(i,:) = [];
    relNeighborVelControls = velocityControls;
    relNeighborVelControls(i,:) = [];
    relNeighborNewPositions = (relNeighborPositions + timeStep * relNeighborVelControls) - centralAgentNewPosition;
    for j = 1:size(agentPositions,1)-1
        if norm(relNeighborNewPositions(j,:)) < 2*agentRadius
            numCollisions = numCollisions + 1;
            radialUnit = -relNeighborNewPositions(j,:)/norm(relNeighborNewPositions(j,:));
            distToChange = 2.05*agentRadius - norm(relNeighborNewPositions(j,:));
            deltaV = radialUnit * distToChange / timeStep;
            velocityControls(i,:) = velocityControls(i,:) + deltaV;
        end
    end
end
newVelocities = velocityControls;
end