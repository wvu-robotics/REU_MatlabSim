%% timeTilCollisionFunc: Time Until Collision
%
% Description: Given a the relative position and velocity of two agents,
% along with the sum of their radii, returns the times until they collide.
%
% Proof of Correctness: Acceleration Controller Formulation and Analysis in
% Google Drive
%
% Parameters:
%   relP: A 1x2 double for the relative position
%       Agent1Pose - Agent2Pose
%   relV: A 1x2 double for the relative velocity
%       Agent1Vel - Agent2Vel
%   radSum: A double for the sum of their radii
%
% Returns:
%   times: A double which is the time until the two agents collide. If they
%       won't collide, then time = NaN. If they are colliding currently,
%       then time = 0.
function time = timeTilCollisionFunc(relP, relV, radSum)
    
    %If their relative velocity is zero, the quadratic formula below
    %doesn't hold, so this case must be considered separately.
    if norm(relV) == 0

        %If they are currently colliding
        if norm(relP) < radSum
            time = 0;
        else %If they aren't
            time = NaN;
        end

    %If the time bounds on the collision period are complex or equal
    elseif dot(relV,relP)^2 - norm(relV)^2 * (norm(relP)^2 - radSum^2) <= 0
        %They won't ever collide
        time = NaN;

    else
        %Calculates the times that bound their collision period.
        %They will collide at any time t in (time1,time2).
        time1 = ( -dot(relV,relP) - sqrt(dot(relV,relP)^2 - norm(relV)^2 * (norm(relP)^2 - radSum^2)) ) / (norm(relV)^2);
        time2 = ( -dot(relV,relP) + sqrt(dot(relV,relP)^2 - norm(relV)^2 * (norm(relP)^2 - radSum^2)) ) / (norm(relV)^2);

        %If both times are non-positive
        if time2 <= 0
            %They are moving away from a collision
            time = NaN;

        %If time1 < 0 < time2
        elseif time1 < 0
            %They are in the middle of a collision
            time = 0;

        %If 0 <= time1 < time2
        else
            %They will collide in the future
            time = time1;
        end
    end
end

