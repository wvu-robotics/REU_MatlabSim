%% Acceleration Controller

%Desription: Given the positions and velocities of all agents, as well as
%their sensingRange, returns an acceleration that first moves the velocity
%quickly out of a VO, then moves it to the prefVelocity.

%Arguments:
%   agentPositions: An numAgentsx2 double where the position of the ith
%       agent is agentPosition(i,:).
%   agentVelocities: An numAgentsx2 double where the velocity of the ith 
%       agent is agentVelocities(i,:).
%   prefVelocities: An numAgentsx2 double where the velocity that the ith
%       agent wants to travel at is prefVelocities(i,:). None of the
%       preferred velocities can be [0,0].
%   sensingRange: A double radius in which an agent can notice others.
%   agentRadius: A double radius of the robots size.
%   transCost: A non-zero double for how costly translational changes in
%       velocity are.

%Returns:
%   accelerationInputs: An numAgentsx2 double where the acceleration of the
%       ith agent is accelerationInputs(i,:).
function accelerationInputs = accelerationControllerFunc(agentPositions, agentVelocities, prefVelocities, sensingRange, agentRadius, transCost)
    %Allocates accelerationInputs as numAgentsx2 double
    accelerationInputs = zeros(size(agentPositions,1),2);

    %For each agent
    for i = 1:size(agentPositions,1)
        
        %Finds the central agents positions and velocity from the overall
        %state
        centralAgentPosition = agentPositions(i,:);
        centralAgentVelocity = agentVelocities(i,:);
        
        %Performs gradient descent toward the preferred velocity
        gradient = gradientVector(prefVelocities(i,:), centralAgentVelocity, transCost);
        if norm(gradient) > 0
            accel = -gradient ./ norm(gradient);
        else
            accel = [0,0];
        end
   
        %To find the neighbors positions and velocities, remove the central
        %agents values from the overall state
        neighborsPositions = agentPositions;
        neighborsPositions(i,:) = [];
        neighborsVelocities = agentVelocities;
        neighborsVelocities(i,:) = [];
        
        %Find the relative positions and velocities to the neighbors.
        relativePoses = neighborsPositions - centralAgentPosition;
        relativeVels = centralAgentVelocity - neighborsVelocities;
        
        %Finds the distance to the neighbors, then removes the agents that
        %aren't in the sensing range from the list of neighbors.
        distToNeighbors = vecnorm(relativePoses, 2, 2);
        neighborsPositions = neighborsPositions(distToNeighbors <= sensingRange,:);
        neighborsVelocities = neighborsVelocities(distToNeighbors <= sensingRange,:);
        relativePoses = relativePoses(distToNeighbors <= sensingRange,:);
        relativeVels = relativeVels(distToNeighbors <= sensingRange,:);
        
        %Finds the times until centralAgent collides with each neighbor
        times = timeTilCollision(centralAgentPosition, centralAgentVelocity, neighborsPositions, neighborsVelocities, agentRadius);
        
        %For each neighbor
        for j = 1:size(times,1)
            %If centralAgent will collide with neighbor j
            if ~isnan(times(j))
                
                %Finds a 1x2 double parallel to the side that should be
                %passed on
                voSideVector = getVOSideVector(relativePoses(j,:), relativeVels(j,:), 2*agentRadius);
                
                %Gets the point on the side of the velocity obstacle that's
                %minimizes ovalMetric(transCost, prefVelocities(i,:), minPoint)
                minPoint = lineArgmin(voSideVector, neighborsVelocities(j,:), prefVelocities(i,:), transCost);
                
                %Adds acceleration toward that point that minimizes
                accel = accel + (minPoint - centralAgentVelocity) ./ (times(j) * norm(minPoint - centralAgentVelocity));
            end
        end
        
        %If it can be, accel is normalized.
        if norm(accel) > 0
            accel = accel ./ norm(accel);
        end
        
        accelerationInputs(i,:) = accel;
    end
end