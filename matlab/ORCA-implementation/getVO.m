%% Function Name: getVO
% [VOAngle, AngleRefToB] = getVO(centralAgentPosition, neighborsPositions, agentRadius)
%
% Description: Determines the size and location of the velocity obstacle
% for a number of neighboring agents
%
% Assumptions: 
%
% Inputs:
%   centralAgentPosition (1x2 vector containing the x,y coordinates of the
%       central agent)
%   neighborsPositions (nx2 vector containing the x,y coordinates of all
%       the central agent's neighbors)
%   agentRadius (radius of all the agents (homogeneous))
%
% Outputs:
%   VOAngle (nx1 vector of half angle of the velocity obstacles for each neighbor in radians)
%   AngleRefToB (nx1 vector of angles from the +x-axis to the relative
%       positions of each neighbor in radians)
%
% $Revision: R2020b$ 
% $Author: Stephen Jacobs$
% $Date: June 4, 2021$
%---------------------------------------------------------

function [VOAngle, AngleRefToB] = getVO(centralAgentPosition, neighborsPositions, agentRadius)

%Determine relative positions and initialize the VOAngle outputs
relativeNeighborPositions = -centralAgentPosition+neighborsPositions;
VOAngle = zeros(size(relativeNeighborPositions,1));

%VOAngle is the half angle of the VO
for i = 1:size(relativeNeighborPositions,1)
    if norm(relativeNeighborPositions(i,:)) > 2*agentRadius
        VOAngle(i) = asin((2*agentRadius)./(norm(relativeNeighborPositions(i,:))));
    else
        VOAngle(i) = pi/2;
    end
end

%AngleRefToB is the angle that the relative position vector to a neighbor
%makes with the positive x-axis. Add pi if the x-component is negative
AngleRefToB = mod(atan2(relativeNeighborPositions(:,2), relativeNeighborPositions(:,1)), 2*pi);

end












