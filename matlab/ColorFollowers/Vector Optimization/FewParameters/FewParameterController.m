function FewParameterController(agent)

%5 parameters:
%1- Affect of weighted density on force to centroid
%2- Affect of weighted density on force ortho to centroid
%3- Affect of color range on force to centroid
%4- Affect of color range on force ortho to centroid
%5- Angle relative to goal angle for density weighting
parameters = agent.getProperty('parameters');

%Need to find the 2 scalars: Angle weighted density, and color range

areaSum = 0;
vectToGoal = agent.goalPose - agent.pose;
unitToGoal = vectToGoal/norm(vectToGoal);
color = mod(atan2(unitToGoal(2),unitToGoal(1)),2*pi);
agent.color = FindColor();

colorSeps = [];
for i = 1:length(agent.measuredAgents)
   
    unitToNeighbor = (agent.measuredAgents(i).pose - agent.pose)/norm(agent.measuredAgents(i).pose - agent.pose);
    goalAngle = mod(atan2(unitToGoal(2),unitToGoal(1)),2*pi);
    neighborAngle = mod(atan2(unitToNeighbor(2),unitToNeighbor(1)),2*pi);
    angleDif = abs(mod((goalAngle+parameters(5)),2*pi) - neighborAngle);
    if angleDif > pi
        angleDif = angleDif - 2*pi;
    end
    
    area = (pi - angleDif)/pi;
    areaSum = areaSum + area;
    
    neighborVectToGoal = agent.measuredAgents(i).goalPose - agent.measuredAgents(i).pose;
    neighborUnitToGoal = neighborVectToGoal/norm(neighborVectToGoal);
    neighborColor = mod(atan2(neighborUnitToGoal(2),neighborUnitToGoal(1)),2*pi);
    colorSep = color - neighborColor;
    if colorSep > pi
        colorSep = 2*pi - colorSep;
    end
    if colorSep < -pi
        colorSep = 2*pi + colorSep;
    end
    colorSeps(i) = colorSep;
end
colorRange = range(colorSeps);
weightedDensity = areaSum/agent.measuringRange^2;
agent.setProperty('colorRange',colorRange);
agent.setProperty('density',weightedDensity);


%need to find 2 vectors
%1- unit to the centroid
%2- unit orhogonal to the centroid

positions = zeros(length(agent.measuredAgents),2);
weights = ones(length(agent.measuredAgents),1);
for i = 1:length(agent.measuredAgents)
    positions(i,:) = agent.measuredAgents(i).pose - agent.pose;
end
centroid = FindCentroid(positions,weights);
orthoToCentroid = [centroid(2), -centroid(1)];

%Now need to do the math to find the next velocity
agent.velocityControl = sum(([parameters(1),parameters(2); parameters(3),parameters(4)]*[colorRange/(2*pi);weightedDensity]+[parameters(6); parameters(7)]).*[centroid;orthoToCentroid])+unitToGoal;
if norm(agent.velocityControl) > agent.maxSpeed
    agent.velocityControl = agent.velocityControl/norm(agent.velocityControl) * agent.maxSpeed;
end

    function centroid = FindCentroid(positions, weights)
        x_bar = sum(weights.*positions(:,1))/sum(weights);
        y_bar = sum(weights.*positions(:,2))/sum(weights);
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

























