%% Get Velocity Obstacle Side Vector

%Description: Given a centralAgent and a neighbor, finds a vector parallel
%to the side of the VO that centralAgent should pass on.

%Proof of Correctness: Analytic Shape of Velocity Obstacles in Google Drive

%Parameters:
%   relP: A 1x2 double for the relative position
%       neighborPostion - centralAgentPosition
%   relV: A 1x2 double for the relative velocity
%       centralAgentVelocity - neighborVelocity
%   radSum: A double for the sum of their radii

%Returns:
%   voSideVector: A 1x2 double pointing in the direction of 
function voSideVector = getVOSideVector(relP, relV, radSum)
    
    %Preallocates an empty 1x2 double
    voSideVector = zeros(1,2);

    %If it's best to pass on the left of neighbor j
    if dot(relV, [-relP(2), relP(1)]) > 0
        voSideVector(1) = relP(1) * sqrt(1 - (radSum / norm(relP))^2) - relP(2) * radSum / norm(relP);
        voSideVector(2) = relP(2) * sqrt(1 - (radSum / norm(relP))^2) + relP(1) * radSum / norm(relP);
    %If it's best to pass on the right of neighbor j
    else
        voSideVector(1) = relP(1) * sqrt(1 - (radSum / norm(relP))^2) + relP(2) * radSum / norm(relP);
        voSideVector(2) = relP(2) * sqrt(1 - (radSum / norm(relP))^2) - relP(1) * radSum / norm(relP);
    end
end