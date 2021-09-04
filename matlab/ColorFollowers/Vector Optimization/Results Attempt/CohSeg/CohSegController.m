function CohSegController(agent)

%Get the parameter array from the agent
group = agent.getProperty('group');

%Need to find the 4 scalers:
%-Number of agents of the same group that are being sensed
%-Number of agents of different groups that are being sensed
%-Distance to the closest neighbor
%-Distance to the center of the map

%Need to find the 4 vectors
%-Centroid of same group
%-Centroid of not same group
%-Vector to closest neighbor
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
otherGroupNum = [0, 0];
sameGroupDen = 0;
otherGroupDen = 0;

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
    else
        otherGroupNum = otherGroupNum + 1 * vectToNeighbor;
        otherGroupDen = otherGroupDen + 1;
    end
end

%Now find the unit vectors of the centroids
sameGroupCentroid = sameGroupNum/sameGroupDen;
sameGroupCentroidUnit = sameGroupCentroid/norm(sameGroupCentroid);
otherGroupCentroid = otherGroupNum/otherGroupDen;
otherGroupCentroidUnit = otherGroupCentroid/norm(otherGroupCentroid);
%Handle the case where there are no neighbors of same or other color
if sameGroupDen == 0
    sameGroupCentroidUnit = [0,0];
end
if otherGroupDen == 0
    otherGroupCentroidUnit = [0,0];
end

if ~isempty(otherGroupCentroid)
    distanceToOtherGroupCentroid = norm(otherGroupCentroid);
else
   distanceToOtherGroupCentroid = 1000; 
end
if isnan(distanceToOtherGroupCentroid)
    distanceToOtherGroupCentroid = 1000;
end

%Collect scalars
numOfSameGroupNeighbors = sameGroupDen;
numOfOtherGroupNeighbors = otherGroupDen;
closestNeighborDist;
distanceToOtherGroupCentroid;
distanceToCenter;
scalers = [numOfSameGroupNeighbors; numOfOtherGroupNeighbors; 10/closestNeighborDist^2; 10/distanceToOtherGroupCentroid^3; distanceToCenter; 1];

%Collect vectors
sameGroupCentroidUnit;
orthoToSameGroupCentroidUnit = [sameGroupCentroidUnit(2), -sameGroupCentroidUnit(1)];
otherGroupCentroidUnit;
orthoToOtherGroupCentroidUnit = [otherGroupCentroidUnit(2), -otherGroupCentroidUnit(1)];
closestNeighborUnit;
unitToCenter;
vectors = [sameGroupCentroidUnit; orthoToSameGroupCentroidUnit; otherGroupCentroidUnit; orthoToOtherGroupCentroidUnit; closestNeighborUnit; unitToCenter];

parameters = [-0.1,  -0.1,     0,     -0.1,  0,    1.8; %sameGroupCentroidUnit
              0.1,      0,     0,     0,  0,   -0.8; %orthoToSameGroupCentroidUnit
              0,      -.1,  -0.1,     -0.1,  0,   +0.3; %otherGroupCentroidUnit
              -0.1,   +0.1,    0,     0,  0,   +1.2; %orthoToOtherGroupCentroidUnit
              0,        0,   -0.3,    0,  0,   +0.4; %closestNeighborUnit
              0,        0,     0,     0,  0.01,    .01]*3; %unitToCenter
              

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

























