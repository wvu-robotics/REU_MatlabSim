%% getVOSideVector: Get Velocity Obstacle Side Vector
% Description: Given a centralAgent and its neighbor, finds a vector
% parallel to the side of the VO that centralAgent should pass on.
%
% Proof of Correctness: Analytic Shape of Velocity Obstacles in Google Drive
%
% Parameters:
%   relP: A 1x2 double for the relative position
%       neighborPostion - centralAgentPosition
%   relV: A 1x2 double for the relative velocity
%       centralAgentVelocity - neighborVelocity
%   radSum: A double for the sum of their radii
%
% Returns:
%   voSideVector: A 1x2 double pointing in the direction of the side of the
%       velocity obstacle.
%   normalVector: A 1x2 double pointing perpendicularly away from the
%       closest side of that velocity obstacle.
%
% Edge Case:
%   norm(relP) < radSum: In this case, the agents are already intersecting,
%       which would mathematically mean that the velocity obstacle expands
%       to the entire 2D plane. Instead of throwing an error when that
%       occurs, getVOSideVector returns
%
%                 [voSideVector = [-relP(2),relP(1)], normalVector = -relP]
%
%       This represents the velocity obstacle when the agents are tangent;
%       it widens into a half plane where the normalVector points directly
%       back at the centralAgent.
function [voSideVector, normalVector] = getVOSideVector(relP, relV, radSum)
    
    %Preallocation
    voSideVector = zeros(1,2);

    %If the agents are already intersecting
    if norm(relP) < radSum
        voSideVector = [-relP(2),relP(1)];
        normalVector = -relP;
    
    %If the agents aren't intersecting and it's best to pass on the left of
    %the neighbor
    elseif dot(relV, [-relP(2), relP(1)]) > 0
        voSideVector(1) = relP(1) * sqrt(1 - (radSum / norm(relP))^2) - relP(2) * radSum / norm(relP);
        voSideVector(2) = relP(2) * sqrt(1 - (radSum / norm(relP))^2) + relP(1) * radSum / norm(relP);
        
        normalVector = [-voSideVector(2),voSideVector(1)];
        
    %If the agents aren't intersecting and it's best to pass on the right
    %of the neighbor
    else
        voSideVector(1) = relP(1) * sqrt(1 - (radSum / norm(relP))^2) + relP(2) * radSum / norm(relP);
        voSideVector(2) = relP(2) * sqrt(1 - (radSum / norm(relP))^2) - relP(1) * radSum / norm(relP);
        
        normalVector = [voSideVector(2),-voSideVector(1)];
    end
end