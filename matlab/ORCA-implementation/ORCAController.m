%% Function Name: ORCAController
% velocityControls = ORCAController(postitionState, velocityState, preferedVelocities, tau, sensingRange)
%
% Description: This function finds the velocity control outputs for a
% number of agents based on the ORCA method
%
% Assumptions: Position space is continuous and the agents are holonomic.
% The velocity space is discritized in this implementation. The agents are
% also homogeneous
%
% Inputs:
%   positionState (X,Y positions of each agent arranged with each each
%       row as one agent)
%   velocityState (Vx, Vy velocities of each robot arranged with each
%       row as one agent)
%   preferedVelocities (Prefered Vx, Vy velocities of each robot arrange
%       with each row as one agent)
%   timeHorizon (Time horizon for guarnteed collision avoidance, smaller is
%       easier but less conservative)
%   sensingRange (Radius at which other agents can be sensed)
%   agentRadius (Radius of agents)
%   maxVelocity
%   velocityDiscritisation
%
% Outputs:
%   velocityControls (Vx, Vy velocity controls of each agent with each
%   row as one agent)
%
% $Revision: R2020b$ 
% $Author: Stephen Jacobs$
% $Date: June 4, 2021$
%---------------------------------------------------------


function velocityControls = ORCAController(positionState, velocityState, preferedVelocities, timeHorizon, sensingRange, agentRadius, maxVelocity, velocityDiscritisation, vOptIsZero, responsibility)

%Initialize the output velocity controls
velocityControls = zeros(size(positionState,1),2);

%Find the discritized set of all velocities
possibleVelocities = -maxVelocity:velocityDiscritisation:maxVelocity;
possibleVelControls = zeros(size(possibleVelocities, 2),2); %vx, vy
for i = 1:length(possibleVelocities)
    for j = 1:length(possibleVelocities)
        possibleVelControls((i-1)*length(possibleVelocities)+j, 1) = possibleVelocities(i);
        possibleVelControls((i-1)*length(possibleVelocities)+j, 2) = possibleVelocities(j);
    end
end

for i = 1:size(positionState,1)    
   %For Each agent, find the central agents positions and velocity from the
   %overall state
   centralAgentPosition = positionState(i, :);
   centralAgentVelocity = velocityState(i, :);
   
   %To find the neighbors positions and velocities, remove the central
   %agents values from the overall state
   neighborsPositions = positionState;
   neighborsPositions(i, :) = [];
   neighborsVelocities = velocityState;
   neighborsVelocities(i, :) = [];
   
   %Find the relative positions to the neighbors from the central agent so
   %that you can see if they are in sensing range
   relPositionOfNeighbors = neighborsPositions - centralAgentPosition;
   distToNeighbors = vecnorm(relPositionOfNeighbors, 2, 2);
   
   %Remove the agents that aren't in the sensing range from the list of
   %neighbors
   neighborsPositions = neighborsPositions(distToNeighbors <= sensingRange,:);
   neighborsVelocities = neighborsVelocities(distToNeighbors <= sensingRange,:);
   relPositionOfNeighbors = relPositionOfNeighbors(distToNeighbors <= sensingRange,:);
   
   %if there are neighbors, use Acceptable Velocity to determine what
   %velocities are acceptable. Else, the velocity control is just the
   %prefered velocity
   if size(neighborsPositions,1) ~= 0
       acceptability = AcceptableVelocity(centralAgentPosition, centralAgentVelocity, neighborsPositions, neighborsVelocities, agentRadius, possibleVelControls, timeHorizon, vOptIsZero,responsibility);
       if sum(acceptability) == 0
           velocityControls(i, :) = [0,0];
           continue;
       end
   else
       velocityControls(i, :) = preferedVelocities(i, :);
       continue;
   end
   
   %Now use the acceptability criteria to narrow down the allowed
   %velocities and pick the best one
   acceptableVelocities = possibleVelControls(acceptability == 1, :);
   distFromPrefered = vecnorm(acceptableVelocities - preferedVelocities(i, :), 2, 2);
   [~, bestVelocityIndex] = min(distFromPrefered);
   velocityControls(i, :) = acceptableVelocities(bestVelocityIndex, :);
end
end
