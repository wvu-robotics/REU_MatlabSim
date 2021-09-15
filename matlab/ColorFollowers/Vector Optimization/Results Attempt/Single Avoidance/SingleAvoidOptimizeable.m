function cost = SingleAvoidOptimizeable(parameters, display)

%Move parameters into a useable format
try
    for i = 1:size(parameters,2)
        name = char(sprintf('param%i',i));
        tempParameters(i) = parameters.(name);
    end
    parameters = tempParameters;
    paramRows = 4;
    paramCols = 3;
    parameters = reshape(parameters, paramRows, paramCols);
catch
end

%% Setting Up Sim
%   World Building
numberOfAgents = 4;
numberOfGroups = 4;
agentRadius = .5;
timeStep = .1;
mapSize = 10;
counter = 0;
maxSpeed = 5;
sensingRange = 16;
timeSteps = 100;

%Setup spawn type
spawnType = 'antipodal';
%spawnType = 'random';

%Set f to appropriate controller handle
f = @SingleAvoidControllerOptimizeable;
%f = @SingleAvoidController;

%Set up the environment
ENV = agentEnv(numberOfAgents,agentRadius,mapSize,timeStep); 

%Spawn the agents using a custom spawn function
[initPositions, goalLocations] = SingleAvoidSpawn(spawnType, numberOfAgents);

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
    ENV.agents(i).createProperty('parameters', parameters);
    ENV.agents(i).measuringRange = sensingRange;
    ENV.agents(i).maxSpeed = maxSpeed;
end 

%% Running Sim
cost = 0;
while(true)
    customTick(ENV, timeStep, display, mapSize);
    counter = counter + 1;
    %fprintf("Time: %i \n",counter)
    if counter > timeSteps
        break
    end
    cost = cost + SingleAvoidCost(ENV.agents);
    %close(figure(2));
%     F(counter) = getframe(gcf);
    
end
%     video = VideoWriter('SingleAvoid2', 'MPEG-4');
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
end
