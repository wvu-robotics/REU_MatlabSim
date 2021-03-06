function cost = VectorOptimizationRun(parameters)

%   World Building
numberOfAgents = 30;
agentRadius = .5;
timeStep = .1;
mapSize = 15;
counter = 0;
maxSpeed = 2;
sensingRange = 5;

% Parameters for the cost function
timeCost = 0.1;
relativeVelCost = 0.01;
collisionCost = 0.1;
connectivityBenefit = -0.01;

%spawnType = 'random';
spawnType = 'opposingGroups';


f = @ColorVectorController;

ENV = agentEnv(numberOfAgents,agentRadius,mapSize,timeStep); 

[initPositions, goalLocations] = SpawnAgents(numberOfAgents, spawnType, mapSize);

ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));
ENV.realTime = false;
ENV.pathVisibility(false)

for i = 1:numberOfAgents
    ENV.agents(i).setController(f);
    ENV.agents(i).createProperty('parameters',parameters);
    ENV.agents(i).measuringRange = sensingRange;
    ENV.agents(i).maxSpeed = maxSpeed;
end 


cost = 0;
while(true)
    ENV.tick;
    counter = counter + 1;
    fprintf("Time: %i \n",counter)
    if counter > 500
       break 
    end
    ENV.agents(1).getTimeStep;
    
    %Evaluate Some of the cost for each time step
    for i = 1:length(ENV.agents)
        if norm(ENV.agents(i).pose - ENV.agents(i).goalPose) > 0.5
            cost = cost + timeCost;
        end
        for j = 1:length(ENV.agents(i).measuredAgents)
            relSpeed = norm(ENV.agents(i).velocity - ENV.agents(i).measuredAgents(j).velocity);
            cost = cost + relSpeed*relativeVelCost;
        end
        cost = cost + connectivityBenefit * length(ENV.agents(i).measuredAgents);
    end
end
fprintf('Collisions: %.0f\n', ENV.collisions);
cost = cost+collisionCost*ENV.collisions;
fprintf('Cost: %.3f\n', cost);


end


