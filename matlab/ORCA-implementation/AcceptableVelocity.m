%% Function Name: AcceptableVelocity
% acceptability = AcceptableVelocity(centralAgentPosition, centralAgentVelocity, neighborsPositions, neighborsVelocities, radius, possibleVelControls)
%
% Description: Determines the acceptability of each velocity in the allowed
%   velocity space considering the velocity obstacles of each agent that
%   needs to be avoided
%
% Assumptions: 
%
% Inputs: 
%   centralAgentPosition (1x2 position vector in x,y coordinates of central
%       agent)
%   centralAgentVelocity (1x2 velocity vector in Vx, Vy of central agent)
%   neighborsPositions (nx2 position vectors of neighbors in x,y
%       coordinates)
%   neighborsVelocities (nx2 velocity vectors of neighbors in Vx,Vy)
%   agentRadius (Radius of all agents)
%   possibleVelControls (discritized velocity space as mx2 matrix)
%   timeHorizon (time for guarnteed colision avoidance)
%   vOptIsZero (boolian for choosing if vOPT is zero or is the current
%       velocity)
%   responsibility (Amount of responsibility each agent takes to get out of
%       the velocity obstacle. Usually 0.5)
%
% Outputs:
%   acceptability (nx1 logial that encodes which velocities in the velocity
%       space are allowed by ORCA)
%
% $Revision: R2020b$
% $Author: Stephen Jacobs$
% $Date: June 4, 2021$
%---------------------------------------------------------

function [acceptability, psi , b, normalVector] = AcceptableVelocity(centralAgentPosition, centralAgentVelocity, neighborsPositions, neighborsVelocities, agentRadius, possibleVelControls, timeHorizon, vOptIsZero, responsibility)

%Initialize Output
acceptability = ones(size(possibleVelControls,1),1);

%Use getVO to find characteristics of all the velocity obstacles imposed on
%the central agent by its neighbors
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

%find normalVectors and uVectors for each velocity obstacle using
%getNormalVector
[normalVector, uVector, noAvoidance] = getNormalVector(relativeOptVel, relPositionOfNeighbors, agentRadius, VOAngle, timeHorizon, AngleRefToB, relativeVel);

%Remove the normalVectors and uVectors for neighbors that don't need to be
%avoided
normalVector(noAvoidance == 1,:) = [];
uVector(noAvoidance == 1, :) = [];

%for each neighbor, update the acceptability of all velocities in the velocity space 
%based on their velocity obstacle
if size(normalVector,1) == 0 %If there are no neighbors that need to be avoided, all velocities are acceptable
    acceptability(:) = 1;
    psi = 0;
    b = 0;
else
    for i = 1:size(normalVector,1) %For each neighbor that needs to be avoided
        %Theta is the positive angle of every normal vector from the positive  x axis
        theta = mod(atan2(normalVector(i,2),normalVector(i,1)),2*pi);
        
        %Psi is the angle of the tangent line of the half plane
        psi(i) = theta + pi/2;
        
        %adjustedVel describes the point that the half plane line goes through
        adjustedVel = centralOptVel + responsibility * uVector(i,:);
        
        %b is the y-intercept of the half plane line
        b(i) = adjustedVel(2) - tan(psi(i)) * adjustedVel(1);
        
        %Check if the discritized velocities are inside or outside the half
        %plane
        for j = 1:size(possibleVelControls,1)
            %If the y component of the normal vector is positive, allowed
            %velocities are above the half plane line and vise versa
            %It is better to start with every velocity allowed and then
            %remove velocities based on the half plane so that you don't
            %overwrite previous ommissions
            if ((normalVector(i,2) > 0) && (possibleVelControls(j,2) < tan(psi(i))*possibleVelControls(j,1)+b(i))) || ((normalVector(i,2) < 0) && (possibleVelControls(j,2) > tan(psi(i))*possibleVelControls(j,1)+b(i)))
                acceptability(j) = 0;
            end
        end
    end
end
psi = {psi};
b = {b};
normalVector = {normalVector};
end














