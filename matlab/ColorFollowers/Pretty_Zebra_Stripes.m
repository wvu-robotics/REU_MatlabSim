close all
clear
clc

numAgents = 6000; %number of agents
mapSize = 100; %x and y dimension of map
environment = zeros(mapSize, mapSize, 6); %X, Y for RGB and "contains agent" and GoalX,GoalY
sensingRadius = 2; %Radius of sensing range for each agent (square sensing range)
neighborToGoalForceRatio = 25;

%Place agents randomly on the map
for i = 1:numAgents
    new = false;
    while not(new)
        randx = randi(mapSize);
        randy = randi(mapSize);
        if environment(randx,randy,4) == 0
           new = true; 
        end
    end
    environment(randx,randy,4) = 1; %The row coordinate is x, the column coordinate is y
end

%Set the goal positions of each agent to a random point on the map
environment(:,:,5:6) = randi(mapSize,[mapSize,mapSize,2]) .* environment(:,:,4);

%Create matrix of indexes
IndexMat = zeros(mapSize, mapSize, 2); 
xIndex = 1:mapSize;
yIndex = (1:mapSize)';
for i = 1:mapSize
   IndexMat(:,i,2) = yIndex;
   IndexMat(i,:,1) = xIndex;
end

%Find unit vector to goal
unitToGoal = (environment(:,:,5:6) - IndexMat)./vecnorm(environment(:,:,5:6) - IndexMat,2,3);

%Find angle of unit vector to goal relative to x-axis
theta = mod(atan2(unitToGoal(:,:,2),unitToGoal(:,:,1)),2*pi);

%Set the color of each agent depending on their theta angle
environment(:,:,1) = ((theta < pi).*(255-theta*255/pi)) + ((theta >= pi).*(theta-pi)*255/pi);
environment(:,:,2) = (and(theta <= 5/3*pi,theta >= 2/3*pi).*(255-(theta-2/3*pi)*255/pi)) + ...
    ((theta > 5/3*pi).*(theta-5/3*pi)*255/pi) + ((theta < 2/3*pi).*((theta+1/3*pi)*255/pi));
environment(:,:,3) = (and(theta <= 4/3*pi,theta >= 1/3*pi).*(theta-1/3*pi)*255/pi) + ...
    ((theta < 1/3*pi).*(255-(theta+2/3*pi)*255/pi)) + ((theta > 4/3*pi).*(255-(theta-4/3*pi)*255/pi));

%Find the sum of the colors of each agent for normalization reasons
colorSum = environment(:,:,1) + environment(:,:,2) + environment(:,:,3);

%display the image showing locations and colors of every agent
imshow(floor(environment(:,:,1:3))./colorSum*1.5.*environment(:,:,4));

%===========================Main Loop==========================%
while true
   tic
   %find number of agents who have reached their goal points
   numAtGoal = sum(sum(vecnorm(environment(:,:,5:6) - IndexMat,2,3) == 0));
   %erase agents who have reached their goal points
   environment(:,:,:) = environment(:,:,:) .* (vecnorm(environment(:,:,5:6) - IndexMat,2,3) ~= 0);
   
   %spawn a new agent for each one that was erased
   for i = 1:numAtGoal
       new = false;
       while not(new)
           randx = randi(mapSize);
           randy = randi(mapSize);
           if environment(randx,randy,4) == 0
               new = true;
           end
       end
       environment(randx,randy,4) = 1; %The row coordinate is x, the column coordinate is y
       %Set new random goal point
       environment(randx,randy,5:6) = randi(mapSize,size(environment(randx,randy,5:6)));
   end
   
   %Update unitToGoal
   unitToGoal = (environment(:,:,5:6) - IndexMat)./vecnorm(environment(:,:,5:6) - IndexMat,2,3);

   %update theta
   theta = mod(atan2(unitToGoal(:,:,2),unitToGoal(:,:,1)),2*pi);
   
   %move agents
   environment = moveAgents(environment, unitToGoal, sensingRadius, neighborToGoalForceRatio, theta, mapSize);
   
   %------------------------------show image----------------------------%
   %Set the color of each agent depending on their theta angle
   environment(:,:,1) = ((theta < pi).*(255-theta*255/pi)) + ((theta >= pi).*(theta-pi)*255/pi);
   environment(:,:,2) = (and(theta <= 5/3*pi,theta >= 2/3*pi).*(255-(theta-2/3*pi)*255/pi)) + ...
       ((theta > 5/3*pi).*(theta-5/3*pi)*255/pi) + ((theta < 2/3*pi).*((theta+1/3*pi)*255/pi));
   environment(:,:,3) = (and(theta <= 4/3*pi,theta >= 1/3*pi).*(theta-1/3*pi)*255/pi) + ...
       ((theta < 1/3*pi).*(255-(theta+2/3*pi)*255/pi)) + ((theta > 4/3*pi).*(255-(theta-4/3*pi)*255/pi));
   
   %Find the sum of the colors of each agent for normalization reasons
   colorSum = environment(:,:,1) + environment(:,:,2) + environment(:,:,3);
   
   %display the image showing locations and colors of every agent
   img = floor(environment(:,:,1:3))./colorSum*1.5.*environment(:,:,4);
   img = imresize(img, 15);
   imshow(img);
   %--------------------------------------------------------------------%
   disp(toc)
end


function environment = moveAgents(environment, unitToGoal, sensingRadius, neighborToGoalForceRatio, theta, mapSize)
force = zeros([mapSize, mapSize, 2]);
force = force + unitToGoal;
for i = 1:mapSize %x coordinate
    for j = 1:mapSize %y coordinate
        for k = -sensingRadius:sensingRadius %x offset in sensing range
            for l = -sensingRadius:sensingRadius %y offset in sensing range
                if i+k > 0 && i+k <= mapSize && j+l > 0 && j+l <= mapSize && k ~= 0 && l ~= 0 %make sure not off the edge of the map and not on central agent
                    %Find the angle to the neighbors relative position
                    angularDistToNeighbor = environment(i+k, j+l, 4) * abs(mod(atan2(j+l,i+k),2*pi) - theta(i, j));
                    %Find the unit vector pointing to that neighbor
                    unitToNeighbor = [k, l]/norm([k, l]);
                    %For each neighbor, add their force. Attraction if
                    %the angular dist is between 0 and 90 and
                    %repulsion if the anglular dist is between 90 and
                    %180 deg
                    force(i, j, 1) = force(i, j, 1)+unitToNeighbor(1)*(pi-angularDistToNeighbor)/pi*neighborToGoalForceRatio;
                    force(i, j, 2) = force(i, j, 2)+unitToNeighbor(2)*(pi-angularDistToNeighbor)/pi*neighborToGoalForceRatio;
                end
            end
        end
        %Now that the force has been added for each agent, we actually
        %need to move the agents around
        %We will try to move to the neighboring square that is closest
        %to the angle their force points, but if there is another agent
        %there, they will try the two adjacent squares
        forceAngle = mod(atan2(force(i,j,2),force(i,j,1)),2*pi);
        
        xBottomWall = false;
        yBottomWall = false;
        xTopWall = false;
        yTopWall = false;
        if i == 1
            xBottomWall = true;
        end
        if j == 1
            yBottomWall = true;
        end
        if i == mapSize
            xTopWall = true;
        end
        if j == mapSize
            yTopWall = true;
        end
        found = false;
        if (forceAngle <= 22.5/180*pi || forceAngle > 337.5/180*pi)
            if ~yTopWall 
                if environment(i, j+1, 4) == 0
                    environment(i, j+1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~xBottomWall && ~yTopWall && ~found
                if environment(i-1, j+1, 4) == 0
                    environment(i-1, j+1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~yTopWall && ~xTopWall && ~found
                if environment(i+1, j+1, 4) == 0
                    environment(i+1, j+1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
        elseif forceAngle <= 67.5/180*pi
            if ~xTopWall && ~yTopWall 
                if environment(i+1, j+1, 4) == 0
                    environment(i+1, j+1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~xTopWall && ~found
                if environment(i+1, j, 4) == 0
                    environment(i+1, j, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~yTopWall && ~found
                if environment(i, j+1, 4) == 0
                    environment(i, j+1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
        elseif forceAngle <= 112.5/180*pi
            if ~xTopWall 
                if environment(i+1, j, 4) == 0
                    environment(i+1, j, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~yBottomWall && ~xTopWall && ~found
                if environment(i+1, j-1, 4) == 0
                    environment(i+1, j-1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~xTopWall && ~yTopWall && ~found
                if environment(i+1, j+1, 4) == 0
                    environment(i+1, j+1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
        elseif forceAngle <= 157.5/180*pi
            if ~yBottomWall && ~xTopWall 
                if environment(i+1, j-1, 4) == 0
                    environment(i+1, j-1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~xTopWall && ~found
                if environment(i+1, j, 4) == 0
                    environment(i+1, j, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~yBottomWall && ~found
                if environment(i, j-1, 4) == 0
                    environment(i, j-1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
        elseif forceAngle <= 202.5/180*pi
            if ~yBottomWall 
                if environment(i, j-1, 4) == 0
                    environment(i, j-1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~xBottomWall && ~yBottomWall && ~found
                if environment(i-1, j-1, 4) == 0
                    environment(i-1, j-1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~yBottomWall && ~xTopWall && ~found
                if environment(i+1, j-1, 4) == 0
                    environment(i+1, j-1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
        elseif forceAngle <= 247.5/180*pi
            if ~xBottomWall && ~yBottomWall 
                if environment(i-1, j-1, 4) == 0
                    environment(i-1, j-1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~yBottomWall && ~found
                if environment(i, j-1, 4) == 0
                    environment(i, j-1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~xBottomWall && ~found
                if environment(i-1, j, 4) == 0
                    environment(i-1, j, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
        elseif forceAngle <= 292.5/180*pi
            if ~xBottomWall 
                if environment(i-1, j, 4) == 0
                    environment(i-1, j, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~xBottomWall && ~yBottomWall && ~found
                if environment(i-1, j-1, 4) == 0
                    environment(i-1, j-1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~yTopWall && ~xBottomWall && ~found
                if environment(i-1, j+1, 4) == 0
                    environment(i-1, j+1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
        elseif forceAngle <= 337.5/180*pi
            if ~yTopWall && ~xBottomWall 
                if environment(i-1, j+1, 4) == 0
                    environment(i-1, j+1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~xBottomWall && ~found
                if environment(i-1, j, 4) == 0
                    environment(i-1, j, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
            if ~yTopWall && ~found
                if environment(i, j+1, 4) == 0
                    environment(i, j+1, :) = environment(i, j, :);
                    environment(i, j, :) = zeros(size(environment(i, j, :)));
                    found = true;
                end
            end
        end
    end
end
end














