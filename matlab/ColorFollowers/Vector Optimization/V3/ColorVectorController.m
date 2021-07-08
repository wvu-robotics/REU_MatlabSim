function ColorVectorController(agent)

unitToGoal = (agent.goalPose - agent.pose)/norm(agent.goalPose - agent.pose);
if norm(agent.goalPose - agent.pose) == 0
    unitToGoal = [0,0];
end
color = mod(atan2(unitToGoal(2),unitToGoal(1)),2*pi);
agent.color = FindColor();

if ~isempty(agent.measuredAgents)
%Collect measured data into useable arrays
neighborPositions = zeros(length(agent.measuredAgents),2);
neighborVelocities = zeros(length(agent.measuredAgents),2);
neighborGoalPose = zeros(length(agent.measuredAgents),2);
for i = 1:length(agent.measuredAgents)
    neighborPositions(i,:) = agent.measuredAgents(i).pose;
    neighborVelocities(i,:) = agent.measuredAgents(i).velocity;
    neighborGoalPose(i,:) = agent.measuredAgents(i).goalPose;
end

%Find own color(angle to goal in rad) and unit to goal
color = mod(atan2(unitToGoal(2),unitToGoal(1)),2*pi);

%Find neighbor colors and units to goal
neighborUnitToGoal = (neighborGoalPose - neighborPositions)./ vecnorm(neighborGoalPose - neighborPositions,2,2);
neighborColor = mod(atan2(neighborUnitToGoal(:,2), unitToGoal(:,1)), 2*pi);

%Find the scalars and vectors
[scalars, vectors] = GetScalarsAndVectors();

%multiply the scalars and vectors by the parameters to get the new velocity
parameters = agent.getProperty('parameters');
%agent.velocityControl = sum(((parameters*scalars)+1).*vectors);
agent.velocityControl = sum(((parameters*scalars)).*vectors)+unitToGoal*50;
if norm(agent.velocityControl) > agent.maxSpeed
   agent.velocityControl = agent.velocityControl/norm(agent.velocityControl)*agent.maxSpeed; 
end

else
    agent.velocityControl = unitToGoal * agent.maxSpeed;
end

%% Functions
    function [scalars, vectors] = GetScalarsAndVectors()
        %Find the set of important scalars
        colorVariance = var(neighborColor);
        agentDensity = (length(neighborColor) + 1)/(pi*agent.measuringRange^2);
        averageUnitToGoal = [mean([neighborUnitToGoal(:,1); unitToGoal(:,1)]),mean([neighborUnitToGoal(:,2); unitToGoal(:,2)])];
        averageColor = mod(atan2(averageUnitToGoal(2),averageUnitToGoal(1)),2*pi);
        colorSepFromMean = abs(averageColor - color);
        distToGoal = norm(agent.goalPose - agent.pose);
        
        %scalars(1,1) = colorVariance;
        scalars(1,1) = agentDensity;
        scalars(2,1) = colorSepFromMean;
        scalars(3,1) = distToGoal;
        
        %Find the set of important Vectors
        unitToGoal;
        %densityGradient = ?
        if size(neighborVelocities,1) > 1
        averageNeighborVelocity = mean(neighborVelocities)/norm(mean(neighborVelocities));
        if norm(mean(neighborVelocities)) == 0
            averageNeighborVelocity = [0,0];
        end
        else
            averageNeighborVelocity = [0,0];
        end

        
        selfColorWeights = abs(neighborColor - color);
        selfColorWeights(selfColorWeights > 2*pi) = 2*pi - selfColorWeights(selfColorWeights > 2*pi);
        selfColorCentroid = FindCentroid(neighborPositions, (pi-selfColorWeights));
        
        oppositeColorWeights = abs(neighborColor - mod(color + pi,2*pi));
        oppositeColorWeights(oppositeColorWeights > 2*pi) = 2*pi - oppositeColorWeights(oppositeColorWeights > 2*pi);
        oppositeColorCentroid = FindCentroid(neighborPositions, (pi-oppositeColorWeights));
        
        averageColorWeights = abs(neighborColor - averageColor);
        averageColorWeights(averageColorWeights > 2*pi) = 2*pi - averageColorWeights(averageColorWeights > 2*pi);
        averageColorCentroid = FindCentroid(neighborPositions, (pi-averageColorWeights));
        
        vectors(1,:) = [unitToGoal(2), -unitToGoal(1)];
        vectors(2,:) = averageNeighborVelocity;
        averageNeighborVelocity(2);
        vectors(3,:) = [averageNeighborVelocity(1,2), -averageNeighborVelocity(1,1)];
        vectors(4,:) = selfColorCentroid;
        vectors(5,:) = [selfColorCentroid(2), -selfColorCentroid(1)];
        vectors(6,:) = oppositeColorCentroid;
        vectors(7,:) = [oppositeColorCentroid(2), -oppositeColorCentroid(1)];
    end
    function centroid = FindCentroid(positions, weights)
        x_bar = sum(weights.*positions(:,1))/sum(weights);
        y_bar = sum(weights.*positions(:,2))/sum(weights);
        x_bar = x_bar - agent.pose(1);
        y_bar = y_bar - agent.pose(2);
        centroid = [x_bar, y_bar]/norm([x_bar, y_bar]);
    end
    function col = FindColor()
        col(1) = ((color < pi).*(1-color*1/pi)) + ((color >= pi).*(color-pi)*1/pi);
        col(2) = (and(color <= 5/3*pi,color >= 2/3*pi).*(1-(color-2/3*pi)*1/pi)) + ...
            ((color > 5/3*pi).*(color-5/3*pi)*1/pi) + ((color < 2/3*pi).*((color+1/3*pi)*1/pi));
        col(3) = (and(color <= 4/3*pi,color >= 1/3*pi).*(color-1/3*pi)*1/pi) + ...
            ((color < 1/3*pi).*(1-(color+2/3*pi)*1/pi)) + ((color > 4/3*pi).*(1-(color-4/3*pi)*1/pi));
            
        colorSum = col(1) + col(2) + col(3);
        col = min([(col./(colorSum)*1.5);1,1,1]);
    end
end








