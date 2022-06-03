close all
clear
clc

%% define simulation params
% Units mi/ft lb min + kt
% 1 kt = 1.151 mph
% 1 mi = 5280 ft
% xy in mi, alti in ft
% Simulation runs at exactly 60x real time.
overallTime = 5; % min
dt = .05; % min
steps = overallTime/dt; 
numAgents = 20;

maxForwardAccel = 20;
maxAlpha = 2*pi;
maxForwardVel = 1.6;
minForwardVel = 1.1;
maxOmega = 2*pi/3;
separation = 5;
cohesion = 1.5;
alignment = 3;
separationWall = 10;
neighborRadius = 1.5;
sinkRate = 500;

agentScale = 2;

% X position, Y position, Width, Rise Rate, Ceiling
thermal = [0,0,8,600,8000];

simParams = [maxForwardAccel, maxAlpha, maxForwardVel, minForwardVel, maxOmega, ...
             separation, cohesion, alignment, separationWall, neighborRadius, sinkRate];

% define agent velocity and position list
agentPositions = zeros(steps+1,numAgents,4);        %x,y,theta, altitude
agentVelocities = zeros(steps+1,numAgents,2);       %forward,turning
%agentTelemetry = [agentPositions;agentVelocities];

%% initialize to random positions
randPosMax = 6;
Ceiling = 8000;
for i = 1:numAgents
    %Initial positions
    agentPositions(1,i,1)= randInRange(-randPosMax,randPosMax);
    agentPositions(1,i,2)= randInRange(-randPosMax,randPosMax);
    agentPositions(1,i,3)= randInRange(0,2*pi);
    agentPositions(1,i,4)= randInRange(Ceiling-2000,Ceiling);
    %Initial velocities
    agentVelocities(1,i,1)=randInRange(1.3,1.3); %minForwardVel,maxForwardVel
    agentVelocities(1,i,2)=randInRange(0,0);%-maxOmega,maxOmega
end

%% setup video and figure
video = VideoWriter('Boids.avi');
video.FrameRate = 1/dt;
open(video);

%fig = figure('Visible','off','units','pixels','position',[0,0,1440,1080]);
fig = figure('Visible','off');

%% Main Loop
tic
for step = 1:steps
    fprintf("Frame %g/%g\n",step,steps);
    clf
    
    % plot current positions
    %xdata = agentPositions(step,:,1);
    %ydata = agentPositions(step,:,2);
    %scatter(xdata,ydata,50,'filled','black');
    currentPositions = (squeeze(agentPositions(step,:,:)))';
    renderBoids(currentPositions,numAgents,Ceiling,agentScale,thermal);
    hold on
    
    graphScale = 1.75;
    wallMag = graphScale*randPosMax;
    xlim([-wallMag wallMag])
    ylim([-wallMag wallMag])
    daspect([1 1 1])
    
    %Simulate each agent
    for agent=1:numAgents
        if agentPositions(step,agent,4) > 0 % agent must not be dead
            %Step through simulation
            [newAccel, newVel, newPos] = stepSim(agentPositions, agentVelocities, step, agent, dt, numAgents, wallMag, simParams,thermal);
            %Update next positions, velocities
            for i=1:4
                agentPositions(step+1,agent,i) = newPos(i);
            end
            for i=1:2
                agentVelocities(step+1,agent,i) = newVel(i);
            end
        end
    end

    %hold simulation to look at
    hold off
    currFrame = getframe(fig);
    writeVideo(video,currFrame);
    %convert this to realtime
    pause(0.0001);
end

close(video);
toc
%% define simulation step function
function [tempAccel, newVel, newPos] = stepSim(positions, velocities, step, agent, dt, numAgents, wallMag, simParams,thermal)
    %This squeeze method returns 2x1 matrices
    agentPos = squeeze(positions(step,agent,1:2));
    agentTheta = positions(step,agent,3);
    agentAlti = positions(step,agent,4);
    agentVel = squeeze(velocities(step,agent,:));
    
    tempAccel = [0;0];
    
    [maxForwardAccel, maxAlpha, maxForwardVel, minForwardVel, maxOmega, separation, cohesion, alignment, separationWall, neighborRadius, sinkRate] = unpack(simParams);
    
    %Iterate through all other agents
    for other = 1:numAgents
        if (other == agent) || (positions(step,other,4) <= 0) % other agent cant be dead
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
        
        accelMag_separation = separation * -1/distToOther^2; %Negative, to go AWAY from the other
        accelMag_cohesion = cohesion * distToOther^2; %Positive, to go TOWARDS the other
        accelMag_alignment = alignment * 1/distToOther^2;
        
        accel = accelMag_separation*diffUnit + accelMag_cohesion*diffUnit + accelMag_alignment*otherVelUnit;
        tempAccel = tempAccel + accel;
    end
    % End interaction

    %Enforce wall separation
    wallPoints = [[wallMag;agentPos(2)],[agentPos(1);wallMag],[-wallMag;agentPos(2)],[agentPos(1);-wallMag]];
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
    % End wall
    
    %Enforce boundary conditions
    forwardUnit = [cos(agentTheta);sin(agentTheta)];
    newAccel_forward = dot(tempAccel,forwardUnit);
    newAccel_theta = 1.0 * sqrt((norm(tempAccel))^2-newAccel_forward^2);
    
    tempAccel3D = [tempAccel(1),tempAccel(2),0];
    forwardUnit3D = [forwardUnit(1),forwardUnit(2),0];
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
    
    newPos = agentPos + newVel(1)*forwardUnit*dt;
    newTheta = agentTheta + newVel(2)*dt;
    newPos(3) = newTheta;
    newPos(4) = agentAlti - sinkRate*dt;

    % giga jank
    if norm(newPos(1),newPos(2)) < thermal(3)/2 && newPos(4) < thermal(5)
        newPos(4) = newPos(4) + thermal(4)*dt;
    end
    %fprintf("Agent %g: Pos(%g,%g,%g) Vel(%g,%g) Accel(%g,%g) WallAccelGlobal(%g,%g)\n",agent,newPos(1),newPos(2),newPos(3),newVel(1),newVel(2),newAccel(1),newAccel(2),wallAccel(1),wallAccel(2));
end
%% Utility Functions
function num = randInRange(a,b)
    num = rand(1)*(b-a) + a;
end

function [maxForwardAccel, maxAlpha, maxForwardVel, minForwardVel, maxOmega, separation, cohesion, alignment, separationWall, neighborRadius, sinkRate] = unpack(simParams)
    maxForwardAccel = simParams(1);
    maxAlpha = simParams(2);
    maxForwardVel = simParams(3);
    minForwardVel = simParams(4);
    maxOmega = simParams(5);
    separation = simParams(6);
    cohesion = simParams(7);
    alignment = simParams(8);
    separationWall = simParams(9);
    neighborRadius = simParams(10);
    sinkRate = simParams(11);
end

%% Render Boids
%Assume positions = 3 x numAgents matrix
function renderBoids(positions,numAgents,Ceiling,agentScale,thermal)
    %Define relative boid shape = x-values...; y-values...
    boidShape = agentScale.*[-.5, .5, -.5;
                 -.5, 0, .5];
    boidShape(1,:) = boidShape(1,:) * 0.4;
    boidShape(2,:) = boidShape(2,:) * 0.3;
    normAltitude = abs(positions(4,:)./Ceiling.*0.6);
    boidColor = hsv2rgb([normAltitude',ones(1,numAgents)',ones(1,numAgents)']);
    rectangle('Position',[-thermal(3)/2,-thermal(3)/2,thermal(3),thermal(3)],'Curvature',1,'FaceColor',[1,.5,.5]);
    for agent=1:numAgents
        if positions(4,agent) > 0
            position = positions(:,agent);
            theta = position(3);
        
            %Use 2D rotation matrix, there might be a better way to do this... https://en.wikipedia.org/wiki/Rotation_matrix
            rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            rotatedBoidShape = rotationMatrix * boidShape;
            globalBoidShape = rotatedBoidShape + position(1:2);

            %Render polygon
            patch(globalBoidShape(1,:),globalBoidShape(2,:),boidColor(agent,:));
            
        end
               
    end
end