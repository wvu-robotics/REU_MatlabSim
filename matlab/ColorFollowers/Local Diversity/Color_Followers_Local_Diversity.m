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

numAgents = 800; %number of agents
mapSize = 100; %x and y dimension of map
sensingRadius = 10; %Radius of sensing range for each agent (square sensing range)
maxNeighborToGoalForceRatio = 0.5; %Ratio of the force from the goal point to the force exerted by one agent of the same color
movePerStepFraction = 8/10;
middleVariance = 1.0;

environment.RGB = zeros(mapSize, mapSize, 3); %X, Y for RGB
environment.occupancy = zeros(mapSize, mapSize); %X, Y for 'contains agent'
environment.goalLocation = zeros(mapSize, mapSize, 2); %X, Y for goal location
imageResize = 10; %Makes the plot bigger

%spawnType = 'RandomSpawn';
%spawnType = 'CoordinatedSpawn';
spawnType = 'OpposingCoordinatedSpawn';

%Create matrix of indexes
IndexMat = zeros(mapSize, mapSize, 2);
yIndex = 1:mapSize;
xIndex = (1:mapSize)';
for i = 1:mapSize
    IndexMat(:,i,1) = xIndex;
    IndexMat(i,:,2) = yIndex;
end

%Spawn Agents
environment = spawnAgents(environment, numAgents, mapSize, spawnType);

%Find unit vector to goal
unitToGoal = (environment.goalLocation(:,:,:) - IndexMat)./vecnorm(environment.goalLocation(:,:,:) - IndexMat,2,3);

%Find angle of unit vector to goal relative to x-axis
theta = mod(atan2(unitToGoal(:,:,2),unitToGoal(:,:,1)),2*pi);

%Set the color of each agent depending on their theta angle
environment.RGB(:,:,1) = ((theta < pi).*(255-theta*255/pi)) + ((theta >= pi).*(theta-pi)*255/pi);
environment.RGB(:,:,2) = (and(theta <= 5/3*pi,theta >= 2/3*pi).*(255-(theta-2/3*pi)*255/pi)) + ...
    ((theta > 5/3*pi).*(theta-5/3*pi)*255/pi) + ((theta < 2/3*pi).*((theta+1/3*pi)*255/pi));
environment.RGB(:,:,3) = (and(theta <= 4/3*pi,theta >= 1/3*pi).*(theta-1/3*pi)*255/pi) + ...
    ((theta < 1/3*pi).*(255-(theta+2/3*pi)*255/pi)) + ((theta > 4/3*pi).*(255-(theta-4/3*pi)*255/pi));

%Find the sum of the colors of each agent for normalization reasons
colorSum = environment.RGB(:,:,1) + environment.RGB(:,:,2) + environment.RGB(:,:,3);

%display the image showing locations and colors of every agent
imshow(floor(environment.RGB(:,:,1:3))./colorSum*1.5.*environment.occupancy(:,:));

%===========================Main Loop==========================%
for t = 1:3000
    tic
    %find number of agents who have reached their goal points
    numAtGoal = sum(sum(vecnorm(environment.goalLocation(:,:,:) - IndexMat,2,3) == 0));
    %erase agents who have reached their goal points
    environment.occupancy(:,:) = environment.occupancy(:,:) .* (vecnorm(environment.goalLocation(:,:,:) - IndexMat,2,3) ~= 0);
    environment.goalLocation(:,:,:) = environment.goalLocation(:,:,:) .* (vecnorm(environment.goalLocation(:,:,:) - IndexMat,2,3) ~= 0);
    
    
    %spawn a new agent for each one that was erased
    environment = spawnAgents(environment, numAtGoal, mapSize, spawnType);
    
    %Update unitToGoal
    unitToGoal = (environment.goalLocation(:,:,:) - IndexMat)./vecnorm(environment.goalLocation(:,:,:) - IndexMat, 2, 3);
    
    %update theta
    theta = mod(atan2(unitToGoal(:,:,2),unitToGoal(:,:,1)),2*pi);
    
    %find force angles
    forceAngle = findForceDirection(environment, unitToGoal, mapSize, sensingRadius, theta, maxNeighborToGoalForceRatio, middleVariance);
    
    %move agents
    [environment, ~] = moveAgents_Local_Diversity(environment, forceAngle, theta, mapSize, movePerStepFraction);
    
    %Update unitToGoal
    unitToGoal = (environment.goalLocation(:,:,:) - IndexMat)./vecnorm(environment.goalLocation(:,:,:) - IndexMat, 2, 3);
    
    %update theta
    theta = mod(atan2(unitToGoal(:,:,2),unitToGoal(:,:,1)),2*pi);
    
    %------------------------------show image----------------------------%
    %Set the color of each agent depending on their theta angle
    environment.RGB(:,:,1) = ((theta < pi).*(255-theta*255/pi)) + ((theta >= pi).*(theta-pi)*255/pi);
    environment.RGB(:,:,2) = (and(theta <= 5/3*pi,theta >= 2/3*pi).*(255-(theta-2/3*pi)*255/pi)) + ...
        ((theta > 5/3*pi).*(theta-5/3*pi)*255/pi) + ((theta < 2/3*pi).*((theta+1/3*pi)*255/pi));
    environment.RGB(:,:,3) = (and(theta <= 4/3*pi,theta >= 1/3*pi).*(theta-1/3*pi)*255/pi) + ...
        ((theta < 1/3*pi).*(255-(theta+2/3*pi)*255/pi)) + ((theta > 4/3*pi).*(255-(theta-4/3*pi)*255/pi));
    
    %Find the sum of the colors of each agent for normalization reasons
    colorSum = environment.RGB(:,:,1) + environment.RGB(:,:,2) + environment.RGB(:,:,3);
    
    %display the image showing locations and colors of every agent
    img = floor(environment.RGB(:,:,:))./colorSum*1.5.*environment.occupancy(:,:);
    img = imresize(img, imageResize);
    imshow(img);
    %--------------------------------------------------------------------%
    disp(toc)
%     
%         F(t) = getframe(gcf);
end

% video = VideoWriter('Crossing_Streams', 'MPEG-4');
% open(video);
% writeVideo(video, F);
% close(video)













