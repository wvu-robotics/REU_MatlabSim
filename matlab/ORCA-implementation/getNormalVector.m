%% Function Name: getNormalVector
% [normalVector, uVector] = getNormalVector(relativeOptVel, relPositionOfNeighbors, agentRadius, VOAngle, timeHorizon)
%
% Description: Determines the normal-vectors and u-vectors for the velocity obstacles for each
% neighboring agent and determines whether or not to attempt to avoid each
% agent
%
% Assumptions: 
%
% Inputs: 
%   relativeOptVel (either [0,0] for each neighbor or the relative velocity of each neighbor
%       agent depending on vOptIsZero
%   relPositionOfNeighbors (nx2 matrix of x,y vectors relative to the
%       central agent)
%   agentRadius (radius of all agents)
%   VOAngle (nx1 vector of half angles for the velocity obstacles for each
%       neighbor)
%   timeHorizon (time for guarenteed collision avoidance)
%   angleRefToB (nx1 vector of angles from the positive x-axis to the
%       relative position vector for each neighbor)
%
% Outputs:
%   normalVector (unit normal that points in the direction to the closest
%       point on the edge of the velocity obstacle for each neighbor)
%   uVector (vector that points from the relative velocity to the nearest
%       point on the edge of the velocity obstacle
%   noAvoidance (boolean. if 1, no need to avoid this neighbor, if 0, the
%       central agent needs to avoid the velocity)
%
% $Revision: R2020b$
% $Author: Stephen Jacobs$
% $Date: June 4, 2021$
%---------------------------------------------------------

function [normalVector, uVector] = getNormalVector(relativeOptVel, relPositionOfNeighbors, agentRadius, VOAngle, timeHorizon, angleReftoB)

%initialize outputs
normalVector = zeros(size(relativeOptVel, 1), 2);
uVector = zeros(size(relativeOptVel, 1), 2);

%find properties of the truncation circle
centerOfTruncationCircle = (relPositionOfNeighbors)/timeHorizon;
radiusOfTruncationCircle = 2*agentRadius/timeHorizon;

%Find the angle of the normal to the the left and right line segments
angleOfNormalLeft = mod((angleReftoB + VOAngle + pi/2), 2*pi);
angleOfNormalRight = mod((angleReftoB - VOAngle - pi/2), 2*pi);


for i = 1:size(relativeOptVel,1) %for each neighbor
    relOptVelAngle = mod( atan2(relativeOptVel(i,2),relativeOptVel(i,1)), 2*pi );
    
    %First determine whether to go left or right and set the angle of normal
    %This if determines if the angle reference to B is left or right of
    %the relative velocity
    if mod(relOptVelAngle - angleReftoB(i),2*pi) < pi
        angleOfNormal = angleOfNormalLeft(i);
    else
        angleOfNormal = angleOfNormalRight(i);
    end
    
    %Then find the rotation matrix cooresponding to that angle and apply it
    %to find the normal vector for going left or right
    rotationMatrix = [cos(angleOfNormal), -sin(angleOfNormal); sin(angleOfNormal), cos(angleOfNormal)];
    normalVector(i,:) = (rotationMatrix*[1;0])';
    
    %The vector border is a vector that points to the place where the
    %truncation circle is tangent to the left or right line segment
    vectorBorder = centerOfTruncationCircle(i,:) + normalVector(i,:)*radiusOfTruncationCircle;

    %Determine whether to actually go (left or right) or (outisde the
    %circle) and find the u vector
    if dot(relativeOptVel(i,:), vectorBorder) - dot(vectorBorder, vectorBorder) >= 0
        %Go left or right becuase the relative velocity is not inside
        %the truncation circle's area of influence
        uVector(i,:) = normalVector(i,:) * -dot(relativeOptVel(i,:), normalVector(i,:));
    else
        %Go out of the circle because the relative velocity is inside
        %the truncation circle's area of influence
        normalVector(i,:) = (relativeOptVel(i,:) - centerOfTruncationCircle(i,:)) / norm(relativeOptVel(i,:) - centerOfTruncationCircle(i,:));
        uVector(i,:) = normalVector(i,:)*(radiusOfTruncationCircle - norm(relativeOptVel(i,:) - centerOfTruncationCircle(i,:)));
    end
end

%if the agents are inside one another (doesn't quite work)
% for i = 1:size(relPositionOfNeighbors,1)
%     if norm(relPositionOfNeighbors(i,:)) < 2*agentRadius
%         normalVector(i,:) = +relPositionOfNeighbors(i,:)/norm(relPositionOfNeighbors(i,:));
%         uVector(i,:) = normalVector(i,:)*.1;
%     end
% end
end