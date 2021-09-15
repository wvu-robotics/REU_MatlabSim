function SearchingOptimController(agent)

%Get the parameter array from the agent
group = agent.getProperty('group');
maxCounter = 200;
if group == 2
    counter = agent.getProperty('counter');
    if counter < maxCounter
        agent.setProperty('counter', counter + 1);
    end
    agent.velocityControl = [0,0];
    agent.color = FindColor();
else
%Need to find the 4 scalers:
%-Distance to closest neighbor
%-Counter of closest search position

%Need to find the 4 vectors
%-Centroid of search locations weighted by counter
%-Nearest search location
%-Vector to closest neighbor

agent.color = FindColor();

%Initialize the closestNeighbor distance to a big number so that the first
%neighbor is immediately the closest
closestNeighborDist = agent.measuringRange;
closestNeighborUnit = [1, 0];

SearchLocationNum = [0, 0];
SearchLocationDen = 0;
counterOfClosest = 0;
distOfClosestLoc = agent.measuringRange;
closestLocUnit = [1, 0];

for i = 1:length(agent.measuredAgents)
    if agent.measuredAgents(i).getProperty('group') == 1
        vectToNeighbor = agent.measuredAgents(i).pose - agent.pose;
        distToNeighbor = norm(vectToNeighbor);
        unitToNeighbor = vectToNeighbor/distToNeighbor;
    
        if distToNeighbor < closestNeighborDist
            closestNeighborDist = distToNeighbor;
            closestNeighborUnit = unitToNeighbor;
        end
    else %the measured agent is a search location
        vectToNeighbor = agent.measuredAgents(i).pose - agent.pose;
        distToNeighbor = norm(vectToNeighbor);
        unitToNeighbor = vectToNeighbor/distToNeighbor;
        if distToNeighbor < distOfClosestLoc
            distOfClosestLoc = distToNeighbor;
            closestLocUnit = unitToNeighbor;
            counterOfClosest = agent.measuredAgents(i).getProperty('counter');
        end
        if distToNeighbor < 2
           agent.measuredAgents(i).setProperty('counter',1); 
        end
        SearchLocationNum = SearchLocationNum + agent.measuredAgents(i).getProperty('counter') * vectToNeighbor;
        SearchLocationDen = SearchLocationDen + agent.measuredAgents(i).getProperty('counter');       
    end
end

%Now find the unit vectors of the centroids
SearchLocationCentroid = SearchLocationNum/SearchLocationDen;
SearchLocationCentroidUnit = SearchLocationCentroid/norm(SearchLocationCentroid);

%Handle the case where there are no neighbors of same or other color
if SearchLocationDen == 0
    SearchLocationCentroidUnit = [0,0];
end

%Collect scalars
distFromCenter = norm(agent.pose);
closestNeighborDist;
counterOfClosest;
scalers = [(distFromCenter/20)^4; 10/closestNeighborDist^2; counterOfClosest; 1];

%Collect vectors
SearchLocationCentroidUnit;
closestNeighborUnit;
closestLocUnit;
currentVel = agent.velocity;
unitToCenter = -agent.pose/distFromCenter;
vectors = [SearchLocationCentroidUnit; closestNeighborUnit; closestLocUnit; currentVel; unitToCenter];

% parameters = [    0,    0,    0,    2;  %SearchLocationCentroidUnit
%                   0,   -1,    0,    0;  %closestNeighborUnit
%                   0,    0, 0.05,  0.5;  %closestLocUnit
%                   0,    0,    0,    5;  %currentVel
%                 0.1,    0,    0,    0]; %unitToCenter
parameters = agent.getProperty('parameters');              

%Now need to do the math to find the next velocity
agent.velocityControl = sum(parameters * scalers .* vectors);
%if norm(agent.velocityControl) > agent.maxSpeed
    agent.velocityControl = agent.velocityControl/norm(agent.velocityControl) * agent.maxSpeed;
%end
end
    function col = FindColor()
        switch group
            case 1
                %col = [0, 0.7, 0.7];
                switch agent.getID()
                    case 1
                        col = [0, 0.7, 0.7];
                    case 2
                        col = [0.7, 0, 0.7];
                    case 3
                        col = [0.7, 0.7, 0];
                    case 4
                        col = [0.7, 0.7, 0.7];
                    case 5
                        col = [0, 0, 1];
                end
            case 2
                %col = [counter/maxCounter, 1 - counter/maxCounter, 0];
                col = [1 1 1];
        end
    end
end


























