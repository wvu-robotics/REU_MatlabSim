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
%   acceptability (nx1 double that encodes which velocities in the velocity
%       space are allowed by ORCA. Each value is the maximum signed
%       distance from an ORCA halfplance. The velocity is guaranteed
%       collision free if it's non-positive.)
%
% $Revision: R2020b$
% $Author: Stephen Jacobs$
% $Date: June 4, 2021$
%---------------------------------------------------------

function [acceptability, psi, b, normalVector] = AcceptableVelocity(centralAgentPosition, centralAgentVelocity, neighborsPositions, neighborsVelocities, agentRadius, possibleVelControls, timeHorizon, vOptIsZero, responsibility)

    %Initialize Output
    acceptability = zeros(size(possibleVelControls,1),1) - Inf;

    %Use getVO to find characteristics of all the velocity obstacles imposed on
    %the central agent by its neighbors
    [VOAngle, AngleRefToB] = getVO(centralAgentPosition, neighborsPositions, agentRadius);

    %Choose optimal velocities and find relative positions and velocities
    centralOptVel = centralAgentVelocity;
    neighborsOptVels = neighborsVelocities;
    if vOptIsZero
        centralOptVel = [0,0];
        neighborsOptVels = zeros(size(neighborsOptVels));
    end
    relativeOptVel = centralOptVel - neighborsOptVels;
    relPositionOfNeighbors = neighborsPositions - centralAgentPosition;

    %find normalVectors and uVectors for each velocity obstacle using
    %getNormalVector
    [normalVector, uVector] = getNormalVector(relativeOptVel, relPositionOfNeighbors, agentRadius, VOAngle, timeHorizon, AngleRefToB);

    %for each neighbor, update the acceptability of all velocities in the velocity space 
    %based on their velocity obstacle
    if size(normalVector,1) == 0 %If there are no neighbors that need to be avoided
        %All velocities are acceptable.
        psi = 0;
        b = 0;
    else
        %For each neighbor that needs to be avoided
        for i = 1:size(normalVector,1)
            %Theta is the positive angle of every normal vector from the positive  x axis
            theta = mod(atan2(normalVector(i,2),normalVector(i,1)),2*pi);

            %Psi is the angle of the tangent line of the half plane
            psi(i) = theta + pi/2;

            %adjustedVel describes the point that the half plane line goes through
            adjustedVel = centralOptVel + responsibility * uVector(i,:);

            %b is the y-intercept of the half plane line
            b(i) = adjustedVel(2) - tan(psi(i)) * adjustedVel(1);

            %Check if the discretized velocities are inside or outside the half
            %plane
            for j = 1:size(possibleVelControls,1)
                acceptability(j) = max( acceptability(j), dot(adjustedVel - possibleVelControls(j,:),normalVector(i,:)) );
            end
        end
    end
    psi = {psi};
    b = {b};
    normalVector = {normalVector};
end














