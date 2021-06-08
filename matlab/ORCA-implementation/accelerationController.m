%% Acceleration Controller

%Desription: Given the positions and velocities of all agents, as well as
%their sensingRange, returns the appropriate acceleration that should be
%applied to each agent.

%Arguments:
%   agentPositions: An Nx2 double where the position of the ith agent 
%   is agentPosition(i,:).
%   agentVelocities: An Nx2 double where the velocity of the ith agent 
%   is agentVelocities(i,:).
%   sensingRange: A double radius in which an agent can notice
%   others.
%   agentRadius: A double radius of the robots size.

%Returns:
%   accelerationInputs: An Nx2 double where the acceleration of the ith
%   agent is accelerationInputs(i,:).
function accelerationInputs = accelerationController(agentPositions, agentVelocities, sensingRange, agentRadius)
    %For each agent
    for i = 1:numberOfAgents
        %For Each agent, find the central agents positions and velocity from the
        %overall state
        centralAgentPosition = agentPositions(i, :);
        centralAgentVelocity = agentVelocities(i, :);
   
        %To find the neighbors positions and velocities, remove the central
        %agents values from the overall state
        neighborsPositions = agentPositions;
        neighborsPositions(i, :) = [];
        neighborsVelocities = agentVelocities;
        neighborsVelocities(i, :) = [];
   
        %Find the relative positions to the neighbors from the central agent so
        %that you can see if they are in sensing range
        relPositionOfNeighbors = neighborsPositions - centralAgentPosition;
        relativeVel = neighborsVelocities - centralAgentVelocity;
        distToNeighbors = vecnorm(relPositionOfNeighbors, 2, 2);
   
        %Removes the agents that aren't in the sensing range from the list 
        %of neighbors
        neighborsPositions = neighborsPositions(distToNeighbors <= sensingRange,:);
        neighborsVelocities = neighborsVelocities(distToNeighbors <= sensingRange,:);
        relPositionOfNeighbors = relPositionOfNeighbors(distToNeighbors <= sensingRange,:);
        
        [VOAngle, angleReftoB] = getVO(centralAgentPosition, neighborsPositions, agentRadius);
        
        %Finds relative velocities of other neighbors
        
        [normalVector, uVector, noAvoidance] = getNormalVector(relativeVels, relPositionOfNeighbors, agentRadius, VOAngle, timeHorizon, angleReftoB, relativeVel);
    end
end