clear
close all;
clc

%% Setting Up Sim
%   World Building
numberOfAgents = 2;
numberOfGroups = 2;
agentRadius = 0.08;
timeStep = .1;
mapSize = 2;
counter = 0;
maxSpeed = 0.1;
sensingRange = 0.25;
connectionRange = 0;
display = true;
timeSteps = 500000;

%Setup spawn type
spawnType = 'antipodal';
%spawnType = 'random';

%Set f to appropriate controller handle
f = @HardwareController;

%Set up the environment
ENV = agentEnv(numberOfAgents,agentRadius,mapSize,timeStep); 

%Setting Up Ros Stuff
ENV.agents(1).setUpPublisher('/turtle5/cmd_vel');
ENV.agents(2).setUpPublisher('/turtle6/cmd_vel');
ENV.agents(1).setUpSubscriber('/vicon/turtle5/turtle5');
ENV.agents(2).setUpSubscriber('/vicon/turtle6/turtle6');

%Spawn the agents using a custom spawn function
[initPositions, goalLocations] = HardwareSpawn(spawnType, numberOfAgents);

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
while(true)
    ENV.tickRos();
    counter = counter + 1;
    fprintf("Time: %i \n",counter)
    if counter > timeSteps
        break
    end
%         F(counter) = getframe(gcf);
end
%     video = VideoWriter('SingleAvoid1', 'MPEG-4');
%     open(video);
%     writeVideo(video, F);
%     close(video)

%% Required Functions
    function customTick(ENV, timeStep, display, mapSize)
            for ii = 1:length(ENV.agents)
                ENV.agents(ii).callMeasurement(ENV);
                ENV.agents(ii).callController;     
                customPhys(ENV, ii, timeStep);
            end
            if display
                customDraw(ENV, mapSize);
            end
    end
    function customDraw(ENV, mapSize)
        figure(1);
        cla;
            hold on
            xlim([-mapSize*3,mapSize*3]);
            ylim([-mapSize*3,mapSize*3]);
            for ii = 1:length(ENV.agents)
                
                RGB = ENV.agents(ii).color;
                plot(ENV.agents(ii).pose(1), ENV.agents(ii).pose(2), '.', 'MarkerEdge', RGB, 'MarkerSize', 25);
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
    
