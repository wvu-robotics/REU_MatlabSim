function cost = FewParamOptimizeable(parameters, display)


for i = 1:size(parameters,2)
    name = char(sprintf('param%i',i));
    newParameters(i) = parameters.(name);
end
parameters = newParameters;

%% Setting Up Sim
%   World Building
numberOfAgents = 20;
agentRadius = .2;
timeStep = .1;
mapSize = 10;
counter = 0;
maxSpeed = 2;
sensingRange = 8;
connectionRange = 2;
newParameters = zeros(5,1);
timeSteps = 500;

%spawnType = 'random';
spawnType = 'opposingGroups';

%Set f to appropriate color handle
f = @FewParameterController;

%Set up the environment
ENV = agentEnv(numberOfAgents,agentRadius,mapSize,timeStep); 

%Spawn the agents using a custom spawn function
[initPositions, goalLocations] = FewParamSpawn(numberOfAgents, spawnType, mapSize);

%set some variables in the environment
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));
ENV.realTime = false;
ENV.pathVisibility(false)

%Set some variables for each agent
for i = 1:numberOfAgents
    ENV.agents(i).setController(f);
    ENV.agents(i).createProperty('parameters',newParameters);
    ENV.agents(i).createProperty('colorRange',0);
    ENV.agents(i).createProperty('density',0);
    ENV.agents(i).measuringRange = sensingRange;
    ENV.agents(i).maxSpeed = maxSpeed;
end 

%% Running Sim
cost = 0;
while(true)
   params(1) = parameters(1);
   params(2) = parameters(2);
   params(3) = parameters(3);
   params(4) = parameters(4);
   params(5) = 0;
   params(6) = parameters(5);
   params(7) = parameters(6); 
   
   for i = 1:length(ENV.agents)
      ENV.agents(i).setProperty('parameters',params);
   end
    customTick(ENV, timeStep, display, f, mapSize);
    counter = counter + 1;
    fprintf("Time: %i \n",counter)
    if counter > timeSteps
       break 
    end
    deltaCost = RelativeVelocityCost(ENV.agents);
    cost = cost+deltaCost;
end

%% Required Functions
    function customTick(ENV, timeStep, display, f, mapSize)
            for ii = 1:length(ENV.agents)
                ENV.agents(ii).callMeasurement(ENV);
                ENV.agents(ii).callController;
                ENV.physics(ii);
                %customPhys(ENV, ii, timeStep);
            end
            if display
                customDraw(ENV, f, mapSize);
            end
    end
    function customDraw(ENV, f, mapSize)
        figure(1);
        cla
            hold on
            xlim([-mapSize,mapSize]);
            ylim([-mapSize,mapSize]);
            for ii = 1:length(ENV.agents)
                
                RGB = ENV.agents(ii).color;
                %RGB = RGB/256;
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