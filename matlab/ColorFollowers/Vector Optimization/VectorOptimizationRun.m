clear
clc
close all

%   World Building
numberOfAgents = 30;
agentRadius = .5;
timeStep = .05;
mapSize = 20;
counter = 0;
maxSpeed = 1;
sensingRange = 5;

%spawnType = 'random';
spawnType = 'opposingGroups';

paramRows = 5;
paramCols = 4;
parameters = (rand([5,4])-0.5)*10;

f = @ColorVectorController;

ENV = agentEnv(numberOfAgents,agentRadius,mapSize,timeStep); 

switch spawnType
    case 'random'
        goalLocations = randi([-mapSize,mapSize],numberOfAgents,2); 
        initPositions = randi([-mapSize,mapSize],numberOfAgents,2);
    case 'opposingGroups'
        goalLocations = zeros(numberOfAgents,2); 
        initPositions = zeros(numberOfAgents,2);
        for i = 1:numberOfAgents
           if mod(i,2) == 1
               goalLocations(i,:) = randi([round(-mapSize),round(-mapSize*2/3)],[1,2]);
               initPositions(i,:) = randi([round(mapSize*2/3),round(mapSize)],[1,2]);
           else
               goalLocations(i,:) = randi([round(mapSize*2/3),round(mapSize)],[1,2]);
               initPositions(i,:) = randi([round(-mapSize),round(-mapSize*2/3)],[1,2]);
           end
        end
end

ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));

for i = 1:numberOfAgents
    ENV.agents(i).setController(f);
    ENV.agents(i).createProperty('parameters',parameters);
    ENV.agents(i).measuringRange = sensingRange;
    ENV.agents(i).maxSpeed = maxSpeed;
end 
ENV.pathVisibility(true);

while(true)
    ENV.tick;
    counter = counter + 1;
    fprintf("Time: %i \n",counter)
    if counter > 2000
       break 
    end
    ENV.agents(1).getTimeStep;
end
ENV.collisions







