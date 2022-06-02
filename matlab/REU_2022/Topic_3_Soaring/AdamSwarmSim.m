close all
clear
clc

%% define simulation params
overallTime = 30; % s
dt = .05; % s 
steps = overallTime/dt; 
numAgents = 100;
agentMaxSpeed = 1;
agentMaxOmega = pi/2;
neighborRadius = 1.5;
%eventually could add a dimension param(2 or more)

% define agent velocity and position list
agentPositions = zeros(steps+1,numAgents,3);        %x,y,theta
agentVelocities = zeros(steps+1,numAgents,2);       %forward,turning

%% initialize to random positions
randPosMax = 6;
for i = 1:numAgents
    %Initial positions
    agentPositions(1,i,1)= randInRange(-randPosMax,randPosMax);
    agentPositions(1,i,2)= randInRange(-randPosMax,randPosMax);
    agentPositions(1,i,3)= randInRange(0,2*pi);
    %Initial velocities
    agentVelocities(1,i,1)=randInRange(0,agentMaxSpeed); 
    agentVelocities(1,i,2)=randInRange(-agentMaxOmega,agentMaxOmega);
end

%% setup video and figure
video = VideoWriter('temp4.avi');
video.FrameRate = 1/dt;
open(video);

fig = figure('Visible','off');

%% run simulation
for step = 1:steps
    fprintf("Frame %g/%g\n",step,steps);
    clf
    
    % plot current positions
    xdata = agentPositions(step,:,1);
    ydata = agentPositions(step,:,2);
    scatter(xdata,ydata,50,'filled','black');
    hold on
    
    graphScale = 1.75;
    wallMag = graphScale*randPosMax;
    xlim([-wallMag wallMag])
    ylim([-wallMag wallMag])
    daspect([1 1 1])
    
    %Simulate each agent
    for agent=1:numAgents
        %Step through simulation
        [newAccel, newVel, newPos] = stepSim(agentPositions, agentVelocities, step, agent, dt, numAgents, neighborRadius, wallMag);
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
    currFrame = getframe(fig);
    writeVideo(video,currFrame);

    %convert this to realtime
    pause(0.0001);
end

close(video);


%% define simulation step function
function [tempAccel, newVel, newPos] = stepSim(positions, velocities, step, agent, dt, numAgents, neighborRadius, wallMag)
    %This squeeze method returns 2x1 matrices
    agentPos = squeeze(positions(step,agent,1:2));
    agentTheta = positions(step,agent,3);
    agentVel = squeeze(velocities(step,agent,:));
    
    tempAccel = [0;0];
    
    maxForwardAccel = 20;
    maxAlpha = pi;
    maxForwardVel = 1.5;
    maxOmega = pi;
    
    separation = 5;
    cohesion = 1.5;
    alignment = 3;
    separationWall = 300;
    
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
        
        accelMag_separation = separation * -1/distToOther^2; %Negative, to go AWAY from the other
        accelMag_cohesion = cohesion * distToOther^2; %Positive, to go TOWARDS the other
        accelMag_alignment = alignment * 1/distToOther^2;
        
        accel = accelMag_separation*diffUnit + accelMag_cohesion*diffUnit + accelMag_alignment*otherVelUnit;
        tempAccel = tempAccel + accel;
    end
    
    %Enforce wall separation
    wallPoints = [[wallMag;agentPos(2)],[agentPos(1);wallMag],[-wallMag;agentPos(2)],[agentPos(1);-wallMag]];
    separationWallAccel = [0;0];
    for i=1:4
       wallPoint = wallPoints(:,i);
       diffPos = wallPoint - agentPos;
       distToWall= norm(diffPos);
       if(distToWall > neighborRadius)
           continue;
       end
       diffUnit = diffPos / distToWall;
       accelMag_separationWall = separationWall * -1/distToWall^4;
       if(accelMag_separationWall > norm(separationWallAccel))
           separationWallAccel = accelMag_separationWall * diffUnit;
       end
    end
    tempAccel = tempAccel + separationWallAccel;
    
    %Enforce boundary conditions
    forwardUnit = [cos(agentTheta);sin(agentTheta)];
    newAccel_forward = dot(tempAccel,forwardUnit);
    newAccel_alpha = 1*sqrt((norm(tempAccel))^2-newAccel_forward^2);
    newAccel = [newAccel_forward; newAccel_alpha];
    
    if(norm(newAccel(1)) > maxForwardAccel)
       newAccel(1) = sign(newAccel(1)) * maxForwardAccel; 
    end
    
    if(norm(newAccel(2)) > maxAlpha)
        newAccel(2) = sign(newAccel(2)) * maxAlpha;
    end
    
    newVel = agentVel + tempAccel*dt;
    if(norm(newVel(1)) > maxForwardVel)
        newVel(1) = sign(newVel(1)) * maxForwardVel;
    end
    
    if(norm(newVel(2)) > maxOmega)
        newVel(2) = sign(newVel(2)) * maxOmega;
    end
    
    newPos = agentPos + newVel(1)*forwardUnit*dt;
    newTheta = agentTheta + newVel(2)*dt;
    newPos(3) = newTheta;
end

%% define velocity function
function vel = v(positions,src,v_0,neighborRadius,agentMaxVel)
    v_gain = [0 ; 0]; %v_gain = [0 ; 0];
    %would like to pull out param processing more

    %calculate distance vectors for all nearby, apply non linearity
    for i = 1:length(positions)
        diffPos = squeeze(positions(1,i,:)) - src; %diffPos = positions(i) - src;
        %implement some nonlinearity here
        mag = norm(diffPos);
        if(mag == 0 || mag > neighborRadius)
            continue;
        end
        unit_diff = diffPos ./ mag;
        
        %lennard-jones, arb constants
        %epsilon = 0.01; 
        %sigma = 10; %best distance
        %out = epsilon * (((sigma/mag)^12)+(-2*((sigma/mag)^6)));

        %other attraction repulsion model         
        F = .7;
        L =  1.8;
        out = (F * exp(-mag/L))-exp(-mag);

        if(out ~= 0)
            v_gain = v_gain + out*unit_diff;
        end
    end

%     decide whether or not to have Vs accrue
    inertia = 0;
    uncapped = (v_0 * inertia) + v_gain;
    uncapped_mag = norm(uncapped);
    if(uncapped_mag > agentMaxVel)
        unit_v = uncapped ./ uncapped_mag;
        vel = unit_v * agentMaxVel;
    else
        vel = uncapped;
    end
end

function num = randInRange(a,b)
    num = rand(1)*(b-a) + a;
end
