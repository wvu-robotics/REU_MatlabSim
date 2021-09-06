clear
close all;
clc

%% Setting Up Sim
%   World Building
numberOfAgents = 5;
numOnGrid = 6^2;
numberOfGroups = 1;
agentRadius = .5;
timeStep = .05;
mapSize = 10;
counter = 0;
maxSpeed = 5;
sensingRange = 16; %16
connectionRange = 2;
newParameters = zeros(5,1);
display = true;
timeSteps = 400;
tail_length = 25;

spawnType = 'random';
%spawnType = 'opposingGroups';

%Set f to appropriate controller handle
f = @SearchingController;

%Set up the environment
ENV = agentEnv(numberOfAgents+numOnGrid,agentRadius,mapSize,timeStep); 

%Spawn the agents using a custom spawn function
[initPositions, searchLocations] = SearchingSpawn(numberOfAgents, spawnType, mapSize);

%Set some variables in the environment
ENV.setAgentVelocities(zeros(numberOfAgents+numOnGrid,2));
ENV.realTime = false;
ENV.pathVisibility(false);
ENV.collisions = true;

%Set some variables for each agent
for i = 1:numberOfAgents
    ENV.agents(i).setController(f);
    ENV.agents(i).createProperty('group',1);
    ENV.agents(i).pose = initPositions(i,:);
    ENV.agents(i).measuringRange = sensingRange;
    ENV.agents(i).maxSpeed = maxSpeed;
end 
for i = numberOfAgents+1:numberOfAgents+numOnGrid
    ENV.agents(i).setController(f);
    ENV.agents(i).createProperty('group', 2);
    ENV.agents(i).createProperty('counter', 200);
    ENV.agents(i).pose = searchLocations(i-numberOfAgents,:);
    ENV.agents(i).measuringRange = 0;
    ENV.agents(i).maxSpeed = 0;
end

%% Running Sim
cost = 0;
while(true)
    cost = customTick(ENV, timeStep, display, mapSize, tail_length, cost);
    counter = counter + 1;
    if mod(counter,10) == 0
        fprintf("Time: %i \n",counter)
    end
    if counter > timeSteps
        disp(cost)
        break
    end
    F(counter) = getframe(gcf);
    
end
%     video = VideoWriter('Searching3', 'MPEG-4');
%     open(video);
%     writeVideo(video, F);
%     close(video)


%% Required Functions
    function cost = customTick(ENV, timeStep, display, mapSize, tail_length, cost)
            for ii = 1:length(ENV.agents)
                ENV.agents(ii).callMeasurement(ENV);
                ENV.agents(ii).callController;     
                customPhys(ENV, ii, timeStep);
                ENV.updateAgentPath(ii,ENV.agents(ii).pose);
                if ENV.agents(ii).getProperty('group') == 2
                    cost = cost+ENV.agents(ii).getProperty('counter');
                end
            end
            if display
                customDraw(ENV, mapSize, tail_length);
            end
    end
    function customDraw(ENV, mapSize, tail_length)
        figure(1);
        cla;
            hold on
            xlim([-mapSize*2,mapSize*2]);
            ylim([-mapSize*2,mapSize*2]);
            for ii = 1:length(ENV.agents)
                
                RGB = ENV.agents(ii).color;
                %RGB = RGB/256;
                plot(ENV.agents(ii).pose(1), ENV.agents(ii).pose(2), '.', 'MarkerEdge', RGB, 'MarkerSize', 25);
                if length(ENV.agents(ii).path(:,1)) > tail_length
                    plot(ENV.agents(ii).path(end-tail_length:end, 1), ENV.agents(ii).path(end-tail_length:end, 2), '.', 'MarkerEdge', RGB);
                else
                    plot(ENV.agents(ii).path(:, 1), ENV.agents(ii).path(:, 2), '.', 'MarkerEdge', RGB);
                end
                set(gca,'Color','k');
            end
            hold off
            pause(0.075)
    end
    function customPhys(ENV,id, timeStep)
        controlVel = ENV.agents(id).velocityControl;
        ENV.agents(id).velocity = controlVel;
        ENV.agents(id).pose = ENV.agents(id).pose + ENV.agents(id).velocity*timeStep;
    end
    
