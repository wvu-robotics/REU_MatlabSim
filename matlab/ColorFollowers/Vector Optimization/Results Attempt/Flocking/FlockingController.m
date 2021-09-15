function FlockingController(agent)

%Get the parameter array from the agent
group = agent.getProperty('group');

%Need to find the 4 scalers:
%-Distance to the closest neighbor
%-Distance to the center of the map

%Need to find the 4 vectors
%-Centroid of neighboring agentst
%-Vector to closest neighbor
%-Vector towards average velocity of neighboring agents
%-Vector to origin

vectToCenter = agent.goalPose - agent.pose;
distanceToCenter = norm(vectToCenter);
unitToCenter = vectToCenter/distanceToCenter;

agent.color = FindColor();

%Initialize the closestNeighbor distance to a big number so that the first
%neighbor is immediately the closest
closestNeighborDist = agent.measuringRange;
closestNeighborUnit = [1, 0];

%initialize same group area and opposite group area
sameGroupNum = [0, 0];
sameGroupDen = 0;
angleList = [mod(atan2(agent.velocity(2), agent.velocity(1)), 2*pi)];
for i = 1:length(agent.measuredAgents)
    
    vectToNeighbor = agent.measuredAgents(i).pose - agent.pose;
    distToNeighbor = norm(vectToNeighbor);
    unitToNeighbor = vectToNeighbor/distToNeighbor;
    
    if distToNeighbor < closestNeighborDist
        closestNeighborDist = distToNeighbor;
        closestNeighborUnit = unitToNeighbor;
    end
    
    %add to the respective areas and locaitons to the centroid numerator
    %and denominators
    if group == agent.measuredAgents(i).getProperty('group')
        sameGroupNum = sameGroupNum + 1 * vectToNeighbor;
        sameGroupDen = sameGroupDen + 1;
    end
    
    angleList(end+1) = mod(atan2(agent.measuredAgents(i).velocity(2), agent.measuredAgents(i).velocity(1)), 2*pi);
end

averageHeading = angle(mean(exp(1i*pi*angleList/180)))*180/pi;
averageHeadingVect = [cos(averageHeading), sin(averageHeading)];

%Now find the unit vectors of the centroids
sameGroupCentroid = sameGroupNum/sameGroupDen;
sameGroupCentroidUnit = sameGroupCentroid/norm(sameGroupCentroid);
%Handle the case where there are no neighbors of same or other color
if sameGroupDen == 0
    sameGroupCentroidUnit = [0,0];
    averageHeadingVect = [0, 0];
end

%Collect scalars
closestNeighborDist;
distanceToCenter;
scalers = [5/(closestNeighborDist^3); (distanceToCenter/50)^6; 1];

%Collect vectors
sameGroupCentroidUnit;
closestNeighborUnit;
unitToCenter;
averageHeadingVect;
currentVelocity = agent.velocity;
vectors = [sameGroupCentroidUnit; closestNeighborUnit; unitToCenter; averageHeadingVect; currentVelocity];

parameters = [   0,    0,    0.1; %sameGroupCentroidUnit
                -2,    0,    0; %closestNeighborUnit
                 0,    1,    0; %unitToCenter
                 0,    0,    5; %averageHeadingVect
                 0,    0,    2]*5; %currentVelocity
              

%Now need to do the math to find the next velocity
agent.velocityControl = sum(parameters * scalers .* vectors);
%if norm(agent.velocityControl) > agent.maxSpeed
    agent.velocityControl = agent.velocityControl/norm(agent.velocityControl) * agent.maxSpeed;
%end


    function col = FindColor()
        switch group
            case 1
                col = [0, 0.7, 0.7];
            case 2
                col = [0.7, 0, 0.7];
            case 3
                col = [0.7, 0.7, 0];
            case 4
                col = [0.5, 0.5, 0.5];
            case 5
                col = [0.75, 0.25, 0.25];
        end
    end
end

























