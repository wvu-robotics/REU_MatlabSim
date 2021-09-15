function PathFollowController(agent)

%Get the parameter array from the agent
group = agent.getProperty('group');

%Need to find the scalers:
%-Distance to the closest neighbor
%-Distance to the center of the map
%-How much closer neighbor is to goal
%-Number of neighbors

%Need to find the 4 vectors
%-Vector to closest neighbor
%-Vector to origin
%-Ortho to origin

vectToCenter = agent.goalPose - agent.pose;
distanceToCenter = norm(vectToCenter);
unitToCenter = vectToCenter/distanceToCenter;

agent.color = FindColor();

%Initialize the closestNeighbor distance to a big number so that the first
%neighbor is immediately the closest
closestNeighborDist = agent.measuringRange;
closestNeighborUnit = [1, 0];

for i = 1:length(agent.measuredAgents)
    
    vectToNeighbor = agent.measuredAgents(i).pose - agent.pose;
    distToNeighbor = norm(vectToNeighbor);
    unitToNeighbor = vectToNeighbor/distToNeighbor;
    
    if distToNeighbor < closestNeighborDist
        closestNeighborDist = distToNeighbor;
        closestNeighborUnit = unitToNeighbor;
        neighborVectToGoal = agent.goalPose - agent.measuredAgents(i).pose;
        relativeGoalDistance = distanceToCenter - norm(neighborVectToGoal);
        thetaDiff = atan2(-neighborVectToGoal(2), -neighborVectToGoal(1)) - atan2(-unitToCenter(2), -unitToCenter(1));
        if thetaDiff > pi
            thetaDiff = thetaDiff - 2*pi;
        elseif thetaDiff <= -pi
            thetaDiff = thetaDiff + 2*pi;
        end
    end
end

%Handle the case where there are no neighbors
if length(agent.measuredAgents) == 0
    closestNeighborDist = 5000;
    relativeGoalDistance = 0;
    thetaDiff = 0;
end


%Collect scalars
closestNeighborDist;
distanceToCenter;
relativeGoalDistance;
thetaDiff;
numNeighbors = length(agent.measuredAgents);
scalers = [closestNeighborDist; distanceToCenter; relativeGoalDistance * thetaDiff; 1];

%Collect vectors
closestNeighborUnit;
orthoToClosestNeighbor = [closestNeighborUnit(2), -closestNeighborUnit(1)];
unitToCenter;
vectors = [closestNeighborUnit; orthoToClosestNeighbor; unitToCenter];

parameters = [ 0.5,   -0.1,     0,    -.5; %closestNeighborUnit
                 0,      0,     1,     0; %orthoToClosestNeighbor
               0.1,      0,     0,  -0.2]*5; %unitToCenter
              

%Now need to do the math to find the next velocity
agent.velocityControl = sum(parameters * scalers .* vectors);
if norm(agent.velocityControl) > agent.maxSpeed
    agent.velocityControl = agent.velocityControl/norm(agent.velocityControl) * agent.maxSpeed;
end

    function col = FindColor()
        switch group
            case 1
                col = [0.7, 0.7, 0];
            case 2
                col = [0.7, 0, 0.7];
            case 3
                col = [0, 0.7, 0.7];
            case 4
                col = [0.5, 0.5, 0.5];
            case 5
                col = [0.75, 0.25, 0.25];
        end
    end
end

























