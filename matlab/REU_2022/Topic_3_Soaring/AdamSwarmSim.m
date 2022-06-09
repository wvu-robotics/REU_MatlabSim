close all
clear
clc

%% define simulation params
overallTime = 60; % s
dt = .02; % s 
steps = overallTime/dt; 
numAgents = 10;

maxForwardAccel = 20;
maxAlpha = 2*pi;
maxForwardVel = 5.0;
minForwardVel = 1.0;
maxOmega = pi;

tempScale = 0.1;
separation = 0.15 * tempScale;
cohesion = 1.2 * tempScale;
alignment = 0.30 * tempScale;
separationWall = 20;

neighborRadius = 3;

simParams = [maxForwardAccel, maxAlpha, maxForwardVel, minForwardVel, maxOmega, separation, cohesion, alignment, separationWall, neighborRadius];

% define agent velocity and position list
agentPositions = zeros(steps+1,numAgents,3);        %x,y,theta
agentVelocities = zeros(steps+1,numAgents,2);       %forward,turning

% define string of parameter info
simParamInfo = sprintf("Sep: %.3f, Coh: %.3f, Align: %.3f, SepWall: %.3f,\n N: %g, Vel: [%.1f,%.1f], NRad: %.1f, dt: %.2f",separation,cohesion,alignment,separationWall,numAgents, minForwardVel,maxForwardVel, neighborRadius,dt);

%% initialize to random positions
randPosMax = 6;
for i = 1:numAgents
    %Initial positions
    agentPositions(1,i,1)= randInRange(-randPosMax,randPosMax);
    agentPositions(1,i,2)= randInRange(-randPosMax,randPosMax);
    agentPositions(1,i,3)= randInRange(0,2*pi);
    %Initial velocities
    agentVelocities(1,i,1)=randInRange(1.3,1.3); %minForwardVel,maxForwardVel
    agentVelocities(1,i,2)=randInRange(0,0);%-maxOmega,maxOmega
end

%% setup video and figure
video = VideoWriter('Output Media/Long50FpsCircles.avi');
video.FrameRate = 1/dt;
open(video);

%fig = figure('Visible','off','units','pixels','position',[0,0,1440,1080]);
simFig = figure('Visible','on');

%% run simulation
for step = 1:steps
    c1 = clock;
    fprintf("Frame %g/%g:  ",step,steps);
    clf
    
    % plot current positions
    %xdata = agentPositions(step,:,1);
    %ydata = agentPositions(step,:,2);
    %scatter(xdata,ydata,50,'filled','black');
    currentPositions = (squeeze(agentPositions(step,:,:)))';
    renderBoids(currentPositions,numAgents, neighborRadius);
    hold on
    
    graphScale = 3;
    wallMag = graphScale*randPosMax;
    xlim([-wallMag wallMag])
    ylim([-wallMag wallMag])
    daspect([1 1 1])
    legend(simParamInfo,'AutoUpdate','off','Location','northoutside');
    
    %Simulate each agent
    for agent=1:numAgents
        %Step through simulation
        [newAccel, newVel, newPos] = stepSim(agentPositions, agentVelocities, step, agent, dt, numAgents, wallMag, simParams);
        %Update next positions, velocities
        for i=1:3
           agentPositions(step+1,agent,i) = newPos(i);
        end
        for i=1:2
            agentVelocities(step+1,agent,i) = newVel(i);
        end
    end

    %hold simulation to look at
    hold off
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);

    %convert this to realtime
    pause(0.0001);
    c2 = clock;
    elapsedTime = c2(6)-c1(6);
    if(elapsedTime < 0)
        elapsedTime = elapsedTime + 60;
    end
    fprintf("%g sec\n",elapsedTime);
end

close(video);


%% define simulation step function
function [tempAccel, newVel, newPos] = stepSim(positions, velocities, step, agent, dt, numAgents, wallMag, simParams)
    %This squeeze method returns 2x1 matrices
    agentPos = squeeze(positions(step,agent,1:2));
    agentTheta = positions(step,agent,3);
    agentVel = squeeze(velocities(step,agent,:));
    
    tempAccel = [0;0];
    
    [maxForwardAccel, maxAlpha, maxForwardVel, minForwardVel, maxOmega, separation, cohesion, alignment, separationWall, neighborRadius] = unpack(simParams);
    
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
    
    %fprintf("Agent %g: Pos(%g,%g,%g) Vel(%g,%g) Accel(%g,%g) WallAccelGlobal(%g,%g)\n",agent,newPos(1),newPos(2),newPos(3),newVel(1),newVel(2),newAccel(1),newAccel(2),wallAccel(1),wallAccel(2));
end

function num = randInRange(a,b)
    num = rand(1)*(b-a) + a;
end

function [maxForwardAccel, maxAlpha, maxForwardVel, minForwardVel, maxOmega, separation, cohesion, alignment, separationWall, neighborRadius] = unpack(simParams)
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
end

%Assume positions = 3 x numAgents matrix
function renderBoids(positions,numAgents, neighborRadius)
    %Define relative boid shape = x-values...; y-values...
    boidShape = [-0.5, 0.5, -0.5;
                 -0.5, 0, 0.5];
    boidShape(1,:) = boidShape(1,:) * 0.4;
    boidShape(2,:) = boidShape(2,:) * 0.3;
    circlePoints = 30;
    neighborCircle = zeros(2,circlePoints);
    angles = linspace(0,2*pi,circlePoints);
    neighborCircle(1,:) = neighborRadius * cos(angles);
    neighborCircle(2,:) = neighborRadius * sin(angles);
    faces = 1:circlePoints;
    
    for agent=1:numAgents
        position = positions(:,agent);
        theta = position(3);
        
        %Use 2D rotation matrix, there might be a better way to do this... https://en.wikipedia.org/wiki/Rotation_matrix
        rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        rotatedBoidShape = rotationMatrix * boidShape;
        globalBoidShape = rotatedBoidShape + position(1:2);
        
        %Render polygon
        patch(globalBoidShape(1,:),globalBoidShape(2,:),'k'); 
        
        %Calculate global circle
        globalCircle = (neighborCircle + position(1:2))';
        patch('Faces',faces,'Vertices',globalCircle,'EdgeColor','green','EdgeAlpha',0.4,'FaceColor','none');
    end
    
end