%% Function Name: getVO
% [VOAngle, AngleRefToB] = getVO(centralAgentPosition, neighborsPositions, agentRadius)
%
% Description: 
%
% Assumptions: 
%
% Inputs:
%
% Outputs:
%
% $Revision: R2020b$ 
% $Author: Stephen Jacobs$
% $Date: June 4, 2021$
%---------------------------------------------------------

function [VOAngle, AngleRefToB] = getVO(centralAgentPosition, neighborsPositions, agentRadius)

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
AngleRefToB = mod(atan(relativeNeighborPositions(:,2)./relativeNeighborPositions(:,1)) + (relativeNeighborPositions(:,1)<0)*pi,2*pi);

end












