close all
clear
clc

%% define simulation params
overallTime = 30; % s
dt = .1; % s 
steps = overallTime/dt; 
numAgents = 100;

numThermals = 0;
thermalSizeLims = [4, 4];
thermalVelLims = [0.2,0.4];
thermalPixels = 1000; %Number of map units across a side

randPosMax = 5;
xdims = [-15,15];
ydims = [-15,15];

maxForwardAccel = 20;
maxAlpha = 2*pi;
maxForwardVel = 5;
minForwardVel = 1.0;
maxOmega = 2*pi;

tempScale = 0.1;
separation = 0.15 * tempScale;
cohesion = 1.0 * tempScale;
alignment = 0.10 * tempScale;
separationWall = 20;
thermalSlow = 2;

neighborRadius = 5;

simParams = [maxForwardAccel, maxAlpha, maxForwardVel, minForwardVel, maxOmega, separation, cohesion, alignment, separationWall, thermalSlow, neighborRadius];
mapX = linspace(xdims(1),xdims(2),thermalPixels);
mapY = linspace(ydims(1),ydims(2),thermalPixels);

% define agent velocity and position list
agentPositions = zeros(steps+1,numAgents,3);        %x,y,theta
agentVelocities = zeros(steps+1,numAgents,2);       %forward,turning

% define thermal position and velocities
thermalPosX = rand(numThermals,1)*(xdims(2)-xdims(1)) + xdims(1);
thermalPosY = rand(numThermals,1)*(ydims(2)-ydims(1)) + ydims(1);
thermalVel = rand(numThermals,2)*(thermalVelLims(2)-thermalVelLims(1)) + thermalVelLims(1);
thermalSize = rand(numThermals,1)*(thermalSizeLims(2)-thermalSizeLims(1)) + thermalSizeLims(1);

%% initialize to random positions
for i = 1:numAgents
    %Initial positions
    agentPositions(1,i,1)= randInRange(-randPosMax,randPosMax);
    agentPositions(1,i,2)= randInRange(-randPosMax,randPosMax);
    agentPositions(1,i,3)= randInRange(0,2*pi);
    %Initial velocities
    agentVelocities(1,i,1)=randInRange(minForwardVel,maxForwardVel); %minForwardVel,maxForwardVel
    agentVelocities(1,i,2)=randInRange(0,0);%-maxOmega,maxOmega
end

%% setup video and figure
video = VideoWriter('Output Media/SlopeField.avi');
video.FrameRate = 1/dt;
open(video);

simFig = figure('Visible','off','units','pixels','position',[0,0,1440,1080]);
%simFig = figure('Visible','on');
subplot(1,2,1);
subplot(1,2,2);
h = get(gcf,'Children');
set(h(1), 'NextPlot','add');
set(h(2), 'NextPlot','add');

%% run simulation
for step = 1:steps
    fprintf("Frame %g/%g\n",step,steps);
    cla
    subplot(1,2,1);
    xlim(xdims);
    ylim(ydims);
    daspect([1 1 1]);
    colorbar;
    cbLimits = [0,1];
    set(gca,'clim',cbLimits);
    
    %% Render thermals
    thermalMap = renderThermals(thermalPixels,numThermals, thermalPosX, thermalPosY, thermalSize, mapX, mapY, xdims, ydims);
    
    %% Render Boids
    currentPositions = (squeeze(agentPositions(step,:,:)))';
    renderBoids(currentPositions,numAgents);
    
    %% Step physics
    %Step thermal physics
    [thermalPosX, thermalPosY, thermalVel] = stepThermalPhysics(thermalPosX, thermalPosY, thermalVel, numThermals, xdims, ydims, dt);
    
    %Simulate each agent
    for agent=1:numAgents
        %Step through simulation
        [newAccel, newVel, newPos] = stepSim(agentPositions, agentVelocities, step, agent, dt, numAgents, xdims, ydims, simParams, thermalMap, thermalPixels);
        %Update next positions, velocities
        for i=1:3
           agentPositions(step+1,agent,i) = newPos(i);
        end
        for i=1:2
            agentVelocities(step+1,agent,i) = newVel(i);
        end
    end
    
    %Draw slope field
    subplot(1,2,2);
    cla
    xlim(xdims);
    ylim(ydims);
    daspect([1 1 1]);
    numSlopes = 30;
    slopeDiffX = (xdims(2)-xdims(1))/numSlopes;
    slopeDiffY = (ydims(2)-ydims(1))/numSlopes;
    drawXPos = zeros(numSlopes^2,1);
    drawYPos = zeros(numSlopes^2,1);
    drawXSlope = zeros(numSlopes^2,1);
    drawYSlope = zeros(numSlopes^2,1);
    i = 1;
    for xSlope = 1:numSlopes
        for ySlope = 1:numSlopes
            slopePos = [slopeDiffX * (xSlope-0.5) + xdims(1); slopeDiffY * (ySlope-0.5) + ydims(1)];
            target = getTargetMovement(slopePos, 0, [0;0], step, -1, numAgents, agentPositions, xdims, ydims, simParams, thermalPixels, thermalMap);
            target = target/norm(target)/10; %Normalize
            
            drawXPos(i) = slopePos(1);
            drawYPos(i) = slopePos(2);
            drawXSlope(i) = target(1);
            drawYSlope(i) = target(2);
            
            i = i+1;
        end
    end
    quiver(drawXPos, drawYPos, drawXSlope, drawYSlope,0.3);
    
    %hold simulation to look at
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);

    %convert this to realtime
    pause(0.0001);
end

close(video);


%% define simulation step function
function [tempAccel, newVel, newPos] = stepSim(positions, velocities, step, agent, dt, numAgents, xdims, ydims, simParams, thermalMap, thermalPixels)
    %This squeeze method returns 2x1 matrices
    agentPos = squeeze(positions(step,agent,1:2));
    agentTheta = positions(step,agent,3);
    agentVel = squeeze(velocities(step,agent,:));
    agentForwardUnit = [cos(agentTheta);sin(agentTheta)];
    
    [maxForwardAccel, maxAlpha, maxForwardVel, minForwardVel, maxOmega, separation, cohesion, alignment, separationWall, thermalSlow, neighborRadius] = unpack(simParams);
    
    tempAccel = getTargetMovement(agentPos, agentTheta, agentVel, step, agent, numAgents, positions, xdims, ydims, simParams, thermalPixels, thermalMap);
    
    %Enforce boundary conditions
    newAccel_forward = dot(tempAccel,agentForwardUnit);
    if(norm(tempAccel) - norm(newAccel_forward) < 1E-6)
        newAccel_theta = 0;
    else
        newAccel_theta = 1.5 * sqrt((norm(tempAccel))^2-newAccel_forward^2);
    end
    
    tempAccel3D = [tempAccel(1),tempAccel(2),0];
    forwardUnit3D = [agentForwardUnit(1),agentForwardUnit(2),0];
    turningCrossProduct = cross(forwardUnit3D,tempAccel3D);
    newAccel_theta = newAccel_theta * sign(turningCrossProduct(3));
    
    newAccel = [newAccel_forward; newAccel_theta];
    
    if(norm(newAccel(1)) > maxForwardAccel)
       newAccel(1) = sign(newAccel(1)) * maxForwardAccel; 
    end
    
    if(norm(newAccel(2)) > maxAlpha)
        newAccel(2) = sign(newAccel(2)) * maxAlpha;
    end
    
    newVel(1) = agentVel(1) + newAccel(1)*dt;
    
    if(newVel(1) > maxForwardVel)
        newVel(1) = maxForwardVel;
    elseif(newVel(1) < minForwardVel)
        newVel(1) = minForwardVel;
    end
    
    newVel(2) = newAccel(2);
    
    if(norm(newVel(2)) > maxOmega)
        newVel(2) = sign(newVel(2)) * maxOmega;
    end
    
    newPos = agentPos + newVel(1)*agentForwardUnit*dt;
    newTheta = agentTheta + newVel(2)*dt;
    newPos(3) = newTheta;
    
    %fprintf("Agent %g: Pos(%g,%g,%g) Vel(%g,%g) Accel(%g,%g) WallAccelGlobal(%g,%g)\n",agent,newPos(1),newPos(2),newPos(3),newVel(1),newVel(2),newAccel(1),newAccel(2),wallAccel(1),wallAccel(2));
end

function target = getTargetMovement(agentPos, agentTheta, agentVel, step, agent, numAgents, positions, xdims, ydims, simParams, thermalPixels, thermalMap)
    tempAccel = [0;0];
    [maxForwardAccel, maxAlpha, maxForwardVel, minForwardVel, maxOmega, separation, cohesion, alignment, separationWall, thermalSlow, neighborRadius] = unpack(simParams);
    
    agentForwardUnit = [cos(agentTheta);sin(agentTheta)];
    
    %Iterate through all other agents
    for other = 1:numAgents
        if (other == agent)
            continue;
        end
        
        otherPos = squeeze(positions(step,other,1:2));
        otherTheta = positions(step,other,3);
        otherVelUnit = [cos(otherTheta);sin(otherTheta)];
        
        diffPos = otherPos - agentPos;
        distToOther = norm(diffPos);
        diffUnit = diffPos / distToOther;
        
        if (distToOther == 0 || distToOther > neighborRadius)
            continue;
        end
        
        distScaled = distToOther/neighborRadius;
        
        accelMag_separation = separation * -1/distScaled^2; %Negative, to go AWAY from the other
        accelMag_cohesion = cohesion * distScaled^2; %Positive, to go TOWARDS the other
        accelMag_alignment = alignment * 1/distScaled^2;
        
        accel = accelMag_separation*diffUnit + accelMag_cohesion*diffUnit + accelMag_alignment*otherVelUnit;
        tempAccel = tempAccel + accel;
    end
    
    %Enforce wall separation
    wallPoints = [[xdims(2);agentPos(2)],[agentPos(1);ydims(2)],[xdims(1);agentPos(2)],[agentPos(1);ydims(1)]];
    wallAccelUnits = [[-1;0],[0;-1],[1;1],[0;1]];
    wallAccel = [0;0];
    for i=1:4
       wallPoint = wallPoints(:,i);
       diffPos = wallPoint - agentPos;
       distToWall= norm(diffPos);
       if(distToWall > neighborRadius)
           continue;
       end
       accelMag_separationWall = separationWall * 1/distToWall^2;
       tempAccel = tempAccel + accelMag_separationWall * wallAccelUnits(:,i);
       wallAccel = wallAccel + accelMag_separationWall * wallAccelUnits(:,i);
    end
    
    %Slow down in thermal
    %First, find position in thermalMap
    thermalPixelDiffX = (xdims(2)-xdims(1))/(thermalPixels-1);
    thermalPixelDiffY = (ydims(2)-ydims(1))/(thermalPixels-1);
    xBin = round((agentPos(1)-xdims(1))/thermalPixelDiffX)+1;
    yBin = round((agentPos(2)-ydims(1))/thermalPixelDiffY)+1;
    xBin = max(1,min(xBin,thermalPixels));
    yBin = max(1,min(yBin,thermalPixels));
    thermalValue = thermalMap(yBin,xBin);
    thermalValue = min(thermalValue,1);
    if(thermalValue > 0.2)
        scatter(agentPos(1),agentPos(2));
    end
    %maxForwardVel = maxForwardVel * (1 - 0.7*thermalValue);
    if(agentVel(1) ~= 0)
        tempAccel = tempAccel - agentForwardUnit * thermalValue * thermalSlow;
    end
    target = tempAccel;
end

function num = randInRange(a,b)
    num = rand(1)*(b-a) + a;
end

function [maxForwardAccel, maxAlpha, maxForwardVel, minForwardVel, maxOmega, separation, cohesion, alignment, separationWall, thermalSlow, neighborRadius] = unpack(simParams)
    maxForwardAccel = simParams(1);
    maxAlpha = simParams(2);
    maxForwardVel = simParams(3);
    minForwardVel = simParams(4);
    maxOmega = simParams(5);
    separation = simParams(6);
    cohesion = simParams(7);
    alignment = simParams(8);
    separationWall = simParams(9);
    thermalSlow = simParams(10);
    neighborRadius = simParams(11);
end

%Assume positions = 3 x numAgents matrix
function renderBoids(positions,numAgents)
    %Define relative boid shape = x-values...; y-values...
    boidShape = [-0.5, 0.5, -0.5;
                 -0.5, 0, 0.5];
    boidShape(1,:) = boidShape(1,:) * 0.4;
    boidShape(2,:) = boidShape(2,:) * 0.3;
    for agent=1:numAgents
        position = positions(:,agent);
        theta = position(3);
        
        %Use 2D rotation matrix, there might be a better way to do this... https://en.wikipedia.org/wiki/Rotation_matrix
        rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        rotatedBoidShape = rotationMatrix * boidShape;
        globalBoidShape = rotatedBoidShape + position(1:2);
        
        %Render polygon
        patch(globalBoidShape(1,:),globalBoidShape(2,:),'k');        
    end
end

function finalThermalMap = renderThermals(thermalPixels,numThermals, thermalPosX, thermalPosY, thermalSize, mapX, mapY, xdims, ydims)
    finalThermalMap = zeros(thermalPixels);
    %Iterate through thermals
    for thermalIndex = 1:numThermals
        thermalPos = [thermalPosX(thermalIndex),thermalPosY(thermalIndex)];
        %Create temporary empty matrix to hold distances from this thermal
        distancesFromThermal = zeros(thermalPixels);
        for row = 1:thermalPixels
            for column = 1:thermalPixels
                %At each position in the matrix, find the corresponding map
                %position and calculate its distance from this thermal
                mapPos = [mapX(column),mapY(row)];
                diffPos = thermalPos - mapPos;
                distancesFromThermal(row,column) = norm(diffPos);
            end
        end
        %Use normal distribution to generate thermal (normpdf)
        distancesFromThermal = distancesFromThermal/thermalSize(thermalIndex);
        tempThermalMap = normpdf(distancesFromThermal)/normpdf(0);
        finalThermalMap = finalThermalMap + tempThermalMap;
    end

    thermalMapImg = imagesc(finalThermalMap,'XData',xdims,'YData',ydims);
    thermalMapImg.AlphaData = 1;
end

function [thermalPosX, thermalPosY, thermalVel] = stepThermalPhysics(thermalPosX, thermalPosY, thermalVel, numThermals, xdims, ydims, dt)
    for thermalIndex = 1:numThermals
        thermalPosX(thermalIndex) = thermalPosX(thermalIndex) + thermalVel(thermalIndex,1)*dt;
        thermalPosY(thermalIndex) = thermalPosY(thermalIndex) + thermalVel(thermalIndex,2)*dt;
        
        if(thermalPosX(thermalIndex) >= xdims(2))
            thermalPosX(thermalIndex) = 2*xdims(2) - thermalPosX(thermalIndex);
            thermalVel(thermalIndex,1) = -thermalVel(thermalIndex,1);
        elseif(thermalPosX(thermalIndex) <= xdims(1))
            thermalPosX(thermalIndex) = 2*xdims(1) - thermalPosX(thermalIndex);
            thermalVel(thermalIndex,1) = -thermalVel(thermalIndex,1);
        end
        
        if(thermalPosY(thermalIndex) >= ydims(2))
            thermalPosY(thermalIndex) = 2*ydims(2) - thermalPosY(thermalIndex);
            thermalVel(thermalIndex,2) = -thermalVel(thermalIndex,2);
        elseif(thermalPosY(thermalIndex) <= ydims(1))
            thermalPosY(thermalIndex) = 2*ydims(1) - thermalPosY(thermalIndex);
            thermalVel(thermalIndex,2) = -thermalVel(thermalIndex,2);
        end
    end
end