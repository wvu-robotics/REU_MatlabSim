clear
close all;
clc

%% Setting Up Sim
%   World Building
numberOfAgents = 100;
numberOfGroups = 1;
agentRadius = .5;
timeStep = .05;
mapSize = 40;
counter = 0;
maxSpeed = 5;
sensingRange = 15; %16
connectionRange = 2;
newParameters = zeros(5,1);
display = true;
timeSteps = 500;

spawnType = 'random';
%spawnType = 'opposingGroups';

%Set f to appropriate controller handle
f = @PathFollowController;

%Set up the environment
ENV = agentEnv(numberOfAgents,agentRadius,mapSize,timeStep); 

%Spawn the agents using a custom spawn function
[initPositions, goalLocations] = PathFollowSpawn(numberOfAgents, spawnType, mapSize);

%Set some variables in the environment
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));
ENV.realTime = false;
ENV.pathVisibility(false);
ENV.collisions = true;

%Set some variables for each agent
for i = 1:numberOfAgents
    ENV.agents(i).setController(f);
    ENV.agents(i).createProperty('group',mod(i,numberOfGroups)+1);
    ENV.agents(i).measuringRange = sensingRange;
    ENV.agents(i).maxSpeed = maxSpeed;
end 

%% Running Sim
cost = 0;
while(true)
    customTick(ENV, timeStep, display, mapSize);
    counter = counter + 1;
    fprintf("Time: %i \n",counter)
    if counter > timeSteps
        break
    end
    F(counter) = getframe(gcf);
    if counter == 10
       saveas(gcf, 'Path_10.jpg'); 
    end
    if counter == 100
       saveas(gcf, 'Path_100.jpg'); 
    end
    if counter == 500
       saveas(gcf, 'Path_500.jpg'); 
    end
end
%     video = VideoWriter('PathFollow3', 'MPEG-4');
%     open(video);
%     writeVideo(video, F);
%     close(video)


%% Required Functions
    function customTick(ENV, timeStep, display, mapSize)
            for ii = 1:length(ENV.agents)
                ENV.agents(ii).callMeasurement(ENV);
                ENV.agents(ii).callController;     
                customPhys(ENV, ii, timeStep);
                ENV.updateAgentPath(ii,ENV.agents(ii).pose);
            end
            if display
                customDraw(ENV, mapSize);
            end
    end
    function customDraw(ENV, mapSize)
        figure(1);
        cla;
            hold on
            xlim([-mapSize*2,mapSize*2]);
            ylim([-mapSize*2,mapSize*2]);
            for ii = 1:length(ENV.agents)
                
                %RGB = ENV.agents(ii).color;
                RGB = [1 1 1];
                plot(ENV.agents(ii).pose(1), ENV.agents(ii).pose(2), '.', 'MarkerEdge', RGB, 'MarkerSize', 25);
                if length(ENV.agents(ii).path(:,1)) > 25
                    plot(ENV.agents(ii).path(end-25:end, 1), ENV.agents(ii).path(end-25:end, 2), '.', 'MarkerEdge', RGB);
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
    
