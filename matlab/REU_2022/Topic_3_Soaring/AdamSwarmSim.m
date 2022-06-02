close all
clear
clc

%% define simulation params
overallTime = 30; % s
dt = .1; % s 
steps = overallTime/dt; 
numAgents = 100;
agentMaxVel = 1;
neighborRadius = 5;
%eventually could add a dimension param(2 or more)

% define agent velocity and position list
agentPositions = zeros(steps+1,numAgents,2);
agentVelocities = zeros(steps+1,numAgents,2);
agentAccelerations = zeros(steps+1,numAgents,2);

%% initialize to random positions
randPosMax = 6;
for i = 1:numAgents
    %Initial positions
    agentPositions(1,i,1)= randInRange(-randPosMax,randPosMax);
    agentPositions(1,i,2)= randInRange(-randPosMax,randPosMax);
    %Initial velocities
    agentVelocities(1,i,1)=randInRange(-agentMaxVel,agentMaxVel); 
    agentVelocities(1,i,2)=randInRange(-agentMaxVel,agentMaxVel);
    %Initial accelerations = 0
end

%% setup video and figure
video = VideoWriter('temp2.avi');
video.FrameRate = 1/dt;
open(video);

fig = figure('Visible','on');

%% run simulation
for step = 1:steps
    fprintf("Frame %g/%g\n",step,steps);
    clf
    currentPositions = agentPositions(step,:,:);
    
    % plot current positions
    xdata = currentPositions(:,:,1);
    ydata = currentPositions(:,:,2);
    scatter(xdata,ydata,50,'filled','black');
    hold on
    
    graphScale = 1.25;
    xlim([-graphScale*randPosMax graphScale*randPosMax])
    ylim([-graphScale*randPosMax graphScale*randPosMax])
    daspect([1 1 1])
    
    %Simulate each agent
    for agent=1:numAgents
        %Step through simulation
        [newAccel, newVel, newPos] = stepSim(agentPositions, agentVelocities, agentAccelerations, step, agent, dt, numAgents, neighborRadius);
        %Update next positions, velocities, and accelerations
        for i=1:2
           agentPositions(step+1,agent,i) = newPos(i);
           agentVelocities(step+1,agent,i) = newVel(i);
           agentAccelerations(step+1,agent,i) = newAccel(i);
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
function [newAccel, newVel, newPos] = stepSim(positions, velocities, accelerations, step, agent, dt, numAgents, neighborRadius)
    %This squeeze method returns 2x1 matrices
    agentPos = squeeze(positions(step,agent,:));
    agentVel = squeeze(velocities(step,agent,:));
    agentAccel = squeeze(accelerations(step,agent,:));
    
    newAccel = [0;0];
    
    maxAccel = 2;
    maxVel = 1;

    %Iterate through all agents
    for other = 1:numAgents
        if (other == agent)
            continue;
        end
        
        otherPos = squeeze(positions(step,other,:));
        
        diffPos = otherPos - agentPos;
        distToOther = norm(diffPos);
        diffUnit = diffPos / distToOther;
        
        if (distToOther == 0 || distToOther > neighborRadius)
            continue;
        end
        
        separation = 1;
        cohesion = 0.8;
        
        accelMag_separation = -1/distToOther^2; %Negative, to go AWAY from the other
        accelMag_cohesion = 1/(distToOther-neighborRadius)^2; %Positive, to go TOWARDS the other
        accelMag = separation*accelMag_separation + cohesion*accelMag_cohesion;
        
        accel = accelMag * diffUnit;
        newAccel = newAccel + accel;
    end
    
    %Enforce boundary conditions
    newAccelMag = norm(newAccel);
    if(newAccelMag > maxAccel)
        newAccel = newAccel / newAccelMag * maxAccel;
    end
    
    newVel = agentVel + newAccel*dt;
    newVelMag = norm(newVel);
    if(newVelMag > maxVel)
        newVel = newVel / newVelMag * maxVel;
    end
    
    newPos = agentPos + newVel*dt;
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
