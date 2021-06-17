close all
clear
clc

%========================Good Presets==============================
% numAgents = 500;
% mapSize = 100;
% sensingRadius = 8;
% neighborToGoalForceRatio = 0.072;
%spawnType = 'RandomSpawn';
%===================================================================
% numAgents = 500;
% mapSize = 100;
% sensingRadius = 10;
% neighborToGoalForceRatio = 0.01;
% spawnType = 'OpposingCoordinatedSpawn';
%==================================================================
%numAgents = 300;
%mapSize = 156;
%sensingRadius = 20;
%neighborToGoalForceRatio = 0.4;
%movePerStepFraction = 8/10;
%spawnType = 'OpposingCoordinatedSpawn';
%==================================================================

numAgents = 300; %number of agents
mapSize = 100; %x and y dimension of map
sensingRadius = 10; %Radius of sensing range for each agent (square sensing range)
neighborToGoalForceRatio = 0.8; %Ratio of the force from the goal point to the force exerted by one agent of the same color
movePerStepFraction = 8/10;

environment = zeros(mapSize, mapSize, 6); %X, Y for RGB and "contains agent" and GoalX,GoalY
imageResize = 10; %Makes the plot bigger

%spawnType = 'RandomSpawn';
%spawnType = 'CoordinatedSpawn';
spawnType = 'OpposingCoordinatedSpawn';

switch spawnType
    case 'RandomSpawn'
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
    case 'CoordinatedSpawn'
      %Place agents in the top left corner of the map
        for i = 1:numAgents
            new = false;
            while not(new)
                randx = randi(round(mapSize/4));
                randy = randi(round(mapSize/4));
                if environment(randx,randy,4) == 0
                    new = true;
                end
            end
            environment(randx,randy,4) = 1; %The row coordinate is x, the column coordinate is y
        end
        
        %Set the goal positions of each agent to a random point on the map
        environment(:,:,5:6) = randi(mapSize/2,[mapSize,mapSize,2]) .* environment(:,:,4); 
        environment(:,:,5:6) = environment(:,:,5:6) + mapSize/2;
    case 'OpposingCoordinatedSpawn'
        for i = 1:numAgents
            new = false;
            while not(new)
                randx = randi(round(mapSize/4));
                randy = randi(round(mapSize/4));
                if mod(i,2) == 0
                   randy = randy + mapSize*3/4; 
                   environment(randx,randy,5:6) = randi(round(mapSize/4),[1,1,2]); 
                   environment(randx,randy,5) = environment(randx,randy,5) + mapSize*3/4;
                else
                   environment(randx,randy,5:6) = randi(round(mapSize/4),[1,1,2]); 
                   environment(randx,randy,5:6) = environment(randx,randy,5:6) + mapSize*3/4;
                end
                if environment(randx,randy,4) == 0
                    new = true;
                end
            end
            environment(randx,randy,4) = 1; %The row coordinate is x, the column coordinate is y
        end
end

%Create matrix of indexes
IndexMat = zeros(mapSize, mapSize, 2); 
yIndex = 1:mapSize;
xIndex = (1:mapSize)';
for i = 1:mapSize
   IndexMat(:,i,1) = xIndex;
   IndexMat(i,:,2) = yIndex;
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
for t = 1:3000
    tic
    %find number of agents who have reached their goal points
    numAtGoal = sum(sum(vecnorm(environment(:,:,5:6) - IndexMat,2,3) == 0));
    %erase agents who have reached their goal points
    environment(:,:,:) = environment(:,:,:) .* (vecnorm(environment(:,:,5:6) - IndexMat,2,3) ~= 0);
    
    %spawn a new agent for each one that was erased
    for i = 1:numAtGoal
        
        new = false;
        switch spawnType
            case 'RandomSpawn'
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
            case 'CoordinatedSpawn'
                new = false;
                while not(new)
                    randx = randi(mapSize/4);
                    randy = randi(mapSize/4);
                    if environment(randx,randy,4) == 0
                        new = true;
                    end
                end
                environment(randx,randy,4) = 1; %The row coordinate is x, the column coordinate is y
                environment(randx,randy,5:6) = randi(mapSize/2,size(environment(randx,randy,5:6)));
                environment(randx,randy,5:6) = environment(randx,randy,5:6) + mapSize/2;
            case 'OpposingCoordinatedSpawn'
                    new = false;
                    while not(new)
                        randx = randi(mapSize/4);
                        randy = randi(mapSize/4);
                        if mod(i,2) == 0
                            randy = randy + mapSize*3/4;
                            environment(randx,randy,5:6) = randi(mapSize/4,[1,1,2]);
                            environment(randx,randy,5) = environment(randx,randy,5) + mapSize*3/4;
                        else
                            environment(randx,randy,5:6) = randi(mapSize/4,[1,1,2]);
                            environment(randx,randy,5:6) = environment(randx,randy,5:6) + mapSize*3/4;
                        end
                        if environment(randx,randy,4) == 0
                            new = true;
                        end
                    end
                    environment(randx,randy,4) = 1; %The row coordinate is x, the column coordinate is y
        end
    end
   
   %Update unitToGoal
   unitToGoal = (environment(:,:,5:6) - IndexMat)./vecnorm(environment(:,:,5:6) - IndexMat, 2, 3);

   %update theta
   theta = mod(atan2(unitToGoal(:,:,2),unitToGoal(:,:,1)),2*pi);
   
   %move agents
   [environment, theta] = moveAgents_Clump_Mitigating(environment, unitToGoal, sensingRadius, neighborToGoalForceRatio, theta, mapSize, movePerStepFraction);
   
   %Update unitToGoal
   unitToGoal = (environment(:,:,5:6) - IndexMat)./vecnorm(environment(:,:,5:6) - IndexMat, 2, 3);

   %update theta
   theta = mod(atan2(unitToGoal(:,:,2),unitToGoal(:,:,1)),2*pi);
   
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
   img = imresize(img, imageResize);
   imshow(img);
   %--------------------------------------------------------------------%
   disp(toc)
   
%    F(t) = getframe(gcf);
end

% video = VideoWriter('Crossing_Streams', 'MPEG-4');
% open(video);
% writeVideo(video, F);
% close(video)













