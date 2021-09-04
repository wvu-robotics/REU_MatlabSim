function HardwareController(agent)

%Get the parameter array from the agent
group = agent.getProperty('group');

%Need to find the 2 scalers:
%-Dist to goal
%-Dist to other agent

%Need to find the 4 vectors
%-To goal
%-Ortho to goal
%-To other agent
%-Ortho to other agent

vectToGoal = agent.goalPose - agent.pose;
distanceToGoal = norm(vectToGoal);
unitToGoal = vectToGoal/distanceToGoal;

if distanceToGoal < 0.05
    agent.velocityControl = [0,0];
    return
end

agent.color = FindColor();

%Initialize the closestNeighbor distance to a big number so that the first
%neighbor is immediately the closest
closestNeighborDist = agent.measuringRange;
closestNeighborUnit = [0, 0];

for i = 1:length(agent.measuredAgents)
    
    vectToNeighbor = agent.measuredAgents(i).pose - agent.pose;
    distToNeighbor = norm(vectToNeighbor);
    unitToNeighbor = vectToNeighbor/distToNeighbor;
    
    if distToNeighbor < closestNeighborDist && not(isempty(agent.measuredAgents))
        closestNeighborDist = distToNeighbor;
        closestNeighborUnit = unitToNeighbor;
    end
end

%Collect scalars
distanceToGoal;
closestNeighborDist;
scalers = [distanceToGoal; 0.5/closestNeighborDist^2; 1];

%Collect vectors
unitToGoal;
orthoToGoal = [unitToGoal(2), -unitToGoal(1)];
closestNeighborUnit;
orthoToNeighbor = [closestNeighborUnit(2), -closestNeighborUnit(1)];

vectors = [unitToGoal; orthoToGoal; closestNeighborUnit; orthoToNeighbor];

parameters = [-0.01,        0,    0.8; %to goal
                  0,     0.05,      0; %ortho to goal
                  0,     -0.05,   -0.05; %to neighbor
                  0,      0.1,      0] * .2; %ortho to neighbor
              

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
            case 6
                col = [0.25, 0.25, 0.75];
            case 7
                col = [0.25, 0.75, 0.25];
            case 8
                col = [0, 0, 1];
        end
    end
end

























