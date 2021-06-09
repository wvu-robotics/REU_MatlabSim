%% Function Name: getNormalVector
% [normalVector, uVector] = getNormalVector(relativeOptVel, relPositionOfNeighbors, agentRadius, VOAngle, timeHorizon)
%
% Description: Determines the normal-vectors and u-vectors with 
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

function [normalVector, uVector, noAvoidance] = getNormalVector(relativeOptVel, relPositionOfNeighbors, agentRadius, VOAngle, timeHorizon, angleReftoB, relativeVel)

%initialize outputs
%disp(size(relativeOptVel, 1));
normalVector = zeros(size(relativeOptVel, 1), 2);
uVector = zeros(size(relativeOptVel, 1), 2);
noAvoidance = zeros(size(relativeOptVel, 1), 1);

%find properties of the truncation circle
centerOfTruncationCircle = (relPositionOfNeighbors)/timeHorizon;
radiusOfTruncationCircle = 2*agentRadius/timeHorizon;

%Find the angle of the normal to the the left and right line segments

%disp(angleReftoB + VOAngle + pi/2);

angleOfNormalLeft = mod((angleReftoB + VOAngle + pi/2), 2*pi);
angleOfNormalRight = mod((angleReftoB - VOAngle - pi/2), 2*pi);


for i = 1:size(relativeVel, 1) %for each neighbor
    %First determine whether to go left or right and set the angle of normal
    if angleReftoB(i) - mod(atan2(relativeVel(i, 2), relativeVel(i, 1)),2*pi) > 0 %!!!need to fix!!!
        angleOfNormal = angleOfNormalRight(i);
    else
        angleOfNormal = angleOfNormalLeft(i);
    end
    %Then find the rotation matrix cooresponding to that angle and apply it
    %to find the normal vector for going left or right
    rotationMatrix = [cos(angleOfNormal), -sin(angleOfNormal); sin(angleOfNormal), cos(angleOfNormal)];
    normalVector(i, :) = (rotationMatrix*[1;0])';
    
    %The vector border is a vector that points to the place where the
    %circle is tangle to the line segments
    vectorBorder = centerOfTruncationCircle(i,:) + normalVector(i,:)*radiusOfTruncationCircle;
    
    %Determine if the relative velocity is inside the velocity obstacle
    if relativeVel(i, 1) == 0 && relativeVel(i,2) == 0 %no relative velocity is always okay
        noAvoidance(i) = true;
    elseif (angleReftoB(i) - VOAngle(i) < 0 || angleReftoB(i) + VOAngle(i) > 2*pi) ...
        && (mod(atan2(relativeVel(i,2), relativeVel(i, 1)), 2*pi) < mod(angleReftoB(i) - VOAngle(i),2*pi) && mod(atan2(relativeVel(i,2), relativeVel(i, 1)), 2*pi) > mod(angleReftoB(i) + VOAngle(i),2*pi))             
        %The velocity obstace straddles the x-axis so check if relVel is
        %outside the angles
            noAvoidance(i) = true;
    elseif ((angleReftoB(i) - VOAngle(i) > 0) && (angleReftoB(i) + VOAngle(i) < 2*pi)) ...
            && (mod(atan2(relativeVel(i,2), relativeVel(i, 1)), 2*pi) < mod(angleReftoB(i) - VOAngle(i),2*pi) || mod(atan2(relativeVel(i,2), relativeVel(i, 1)), 2*pi) > mod(angleReftoB(i) + VOAngle(i),2*pi)) 
        %The velocity obstace doesnt straddl the x-axis so check if relVel is
        %outside the angles
            noAvoidance(i) = true;
    elseif (norm(relativeVel(i,:) - centerOfTruncationCircle(i,:)) > radiusOfTruncationCircle) && (norm(relativeVel(i,:)) < norm(vectorBorder))
        %These are relative velocities inside the angle but close to origin
        noAvoidance(i) = true;
    end    

    %Determine whether to actually go left or right or outisde the circle
    %and find the u vector
    if dot(relativeVel(i, :), vectorBorder) - dot(vectorBorder, vectorBorder) > 0 %Go left or right
        uVector(i, :) = normalVector(i, :) * abs(dot(relativeVel(i, :), normalVector(i, :)));
    else %Go out of the circle
        normalVector(i ,:) = (relativeVel(i, :) - centerOfTruncationCircle(i,:))/norm(relativeVel(i, :) - centerOfTruncationCircle(i,:));
        uVector(i, :) = normalVector(i, :)*(radiusOfTruncationCircle - norm(relativeVel(i, :) - centerOfTruncationCircle(i,:)));
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




















