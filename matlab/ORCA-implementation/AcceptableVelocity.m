%% Function Name: AcceptableVelocity
% acceptability = AcceptableVelocity(centralAgentPosition, centralAgentVelocity, neighborsPositions, neighborsVelocities, radius, possibleVelControls)
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

function acceptability = AcceptableVelocity(centralAgentPosition, centralAgentVelocity, neighborsPositions, neighborsVelocities, agentRadius, possibleVelControls, timeHorizon, vOptIsZero, responsibility)

%Initialize Output
acceptability = ones(size(possibleVelControls,1),1);

%Use getVO to find characteristics of all the velocity obstacles
[VOAngle, AngleRefToB] = getVO(centralAgentPosition, neighborsPositions, agentRadius);

%Choose optimal velocities and find relative positions and velocities
centralOptVel = centralAgentVelocity;
neighborsOptVels = neighborsVelocities;
relativeVel = centralAgentVelocity - neighborsVelocities;
if vOptIsZero
    centralOptVel = [0,0];
    neighborsOptVels = zeros(size(neighborsOptVels));
end
relativeOptVel = centralOptVel - neighborsOptVels;
relPositionOfNeighbors = neighborsPositions - centralAgentPosition;


%find normalVectors and uVectors for each velocity obstacle
[normalVector, uVector, noAvoidance] = getNormalVector(relativeOptVel, relPositionOfNeighbors, agentRadius, VOAngle, timeHorizon, AngleRefToB, relativeVel);

%Remove the normalVectores and uVectors for neighbors that don't need to be
%avoided
normalVector(noAvoidance == 1,:) = [];
uVector(noAvoidance == 1, :) = [];

%for each neighbor, update the acceptability based on their velocity
%obstacle
if size(normalVector,1) == 0
    acceptability(:) = 1;
else
    for i = 1:size(normalVector,1)
        %Theta is angle of normal vector
        theta = atan(normalVector(i,2)./normalVector(i,1));
        if normalVector(i,1) < 0
            theta = theta + pi;
        end
        
        %Psi is the angle of the tangent line of the half plane
        psi = theta + pi/2;
        
        %adjustedVel describes the point that the half plane line goes through
        adjustedVel = centralOptVel + responsibility*uVector(i,:);
        
        %b is the y-intercept of the half plane line
        b = adjustedVel(2) - tan(psi) * adjustedVel(1);
        
        %Check if the discritized velocities are inside or outside the half
        %plane
        for j = 1:size(possibleVelControls,1)
            if ((normalVector(i,2) > 0) && (possibleVelControls(j,2) < tan(psi)*possibleVelControls(j,1)+b)) || ((normalVector(i,2) < 0) && (possibleVelControls(j,2) > tan(psi)*possibleVelControls(j,1)+b))
                acceptability(j) = 0;
            end
        end
    end
end
end














