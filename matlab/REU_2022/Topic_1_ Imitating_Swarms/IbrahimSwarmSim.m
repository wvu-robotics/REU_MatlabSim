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
agentVels = zeros(steps+1,numAgents,2);

%% initialize to random positions
randPosMax = 6;
for i = 1:numAgents
    agentPositions(1,i,1)= randInRange(-randPosMax,randPosMax);
    agentPositions(1,i,2)= randInRange(-randPosMax,randPosMax);
%     initial v conditions
    agentVels(1,i,1)=randInRange(-agentMaxVel,agentMaxVel); 
    agentVels(1,i,2)=randInRange(-agentMaxVel,agentMaxVel);
end

%% run simulation
for step = 1:steps
    clf
    currentPositions = agentPositions(step,:,:);
    % plot current positions
    xdata = currentPositions(:,:,1);
    ydata = currentPositions(:,:,2);
    scatter(xdata,ydata,50,'filled','black');
    hold on

    graphScale = 1.75;
    xlim([-graphScale*randPosMax graphScale*randPosMax])
    ylim([-graphScale*randPosMax graphScale*randPosMax])
    
    %update velocities, then integrate for position
    for agent=1:numAgents
        %matlab indexing is weird
        agentPos = squeeze(currentPositions(:,agent,:));
        agentVel = squeeze(agentVels(step,agent,:));

        %written out because array assignment is not supported
        newVel = v(currentPositions,agentPos,agentVel,neighborRadius, agentMaxVel);
        agentVels(step+1,agent,1) = newVel(1);
        agentVels(step+1,agent,2) = newVel(2);

        %note velocities are framed in termed of last movement, as opposed
        %to future

        newPos = agentVels(step+1,agent)*dt + agentPos;
        agentPositions(step+1,agent,1)= newPos(1);
        agentPositions(step+1,agent,2)= newPos(2);
        
    end

    %hold simulation to look at

    %convert this to realtime
    pause(dt);
end


%% define velocity function
function vel = v(positions,src,v_0,neighborRadius,agentMaxVel)
    v_gain = [0 ; 0];
    %would like to pull out param processing more

    %calculate distance vectors for all nearby, apply non linearity
    for i = 1:length(positions)
        diffPos = positions(i) - src;
        %implement some nonlinearity here
        mag = norm(diffPos);
        if(mag == 0 || mag > neighborRadius)
            continue;
        end
        unit_diff = diffPos ./ mag;
        
        %lennard-jones, arb constants
%         epsilon = 1; 
%         sigma = 10; %best distance
%         out = epsilon * (((sigma/mag)^12)+(-2*((sigma/mag)^6)));

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
    uncapped = (v_0* inertia) + v_gain;
    uncapped_mag = norm(uncapped);
    if(uncapped_mag >agentMaxVel)
        unit_v = uncapped ./ uncapped_mag;
        vel = unit_v * agentMaxVel;
    else
        vel = uncapped;
    end
end

function num = randInRange(a,b)
    num = rand(1)*(b-a) + a;
end

