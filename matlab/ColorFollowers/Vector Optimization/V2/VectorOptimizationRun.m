function cost = VectorOptimizationRun(parameters, display, timeSteps)

close(figure(2));
for i = 1:size(parameters,2)
    name = char(sprintf('param%i',i));
    newParameters(i) = parameters.(name);
end
newParameters = reshape(newParameters, [5,4]);
%   World Building
numberOfAgents = 15;
agentRadius = .5;
timeStep = .1;
mapSize = 10;
counter = 0;
maxSpeed = 2;
sensingRange = 8;
connectionRange = 2;

% Parameters for the cost function
distToGoalSlopeCost = 1.0;
relativeVelCost = 2; %Cost if high relative velocity and on top of one another
relativeVelDistSlope = -(relativeVelCost*1.5)/sensingRange; %Cost of high relative velocity decreases with distance
collisionCost = 0.01;
connectivityBenefit = -0.01;

%spawnType = 'random';
spawnType = 'opposingGroups';

%Set f to appropriate color handle
f = @ColorVectorController;

%Set up the environment
ENV = agentEnv(numberOfAgents,agentRadius,mapSize,timeStep); 

%Spawn hte agents using a custom spawn function
[initPositions, goalLocations] = SpawnAgents(numberOfAgents, spawnType, mapSize);

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
    ENV.agents(i).measuringRange = sensingRange;
    ENV.agents(i).maxSpeed = maxSpeed;
end 


cost = 0;
while(true)
    customTick();
    numCol = checkCollide();
    counter = counter + 1;
    fprintf("Time: %i \n",counter)
    if counter > timeSteps
       break 
    end
    
    %Evaluate Some of the cost for each time step
    for i = 1:length(ENV.agents)
        distToGoal = norm(ENV.agents(i).pose - ENV.agents(i).goalPose);
        cost = cost + distToGoal * distToGoalSlopeCost;
        for j = 1:length(ENV.agents(i).measuredAgents)
            distToNeighbor = norm(ENV.agents(i).pose - ENV.agents(i).measuredAgents(j).pose);
            relSpeed = norm(ENV.agents(i).velocity - ENV.agents(i).measuredAgents(j).velocity);
            cost = cost + max(relSpeed*(relativeVelCost+distToNeighbor*relativeVelDistSlope),0);
        end
        for j = 1:length(ENV.agents(i).measuredAgents)
            distToNeighbor = norm(ENV.agents(i).pose - ENV.agents(i).measuredAgents(j).pose);
            if distToNeighbor < connectionRange
                cost = cost + connectivityBenefit * length(ENV.agents(i).measuredAgents);
            end
        end
    end
end
fprintf('Collisions: %.0f\n', ENV.collisions);
cost = cost+collisionCost*numCol;
fprintf('Cost: %.3f\n', cost);

    function customTick()
            for ii = 1:length(ENV.agents)
                ENV.agents(ii).callMeasurement(ENV);
                ENV.agents(ii).callController;     
                customPhys(ii);
            end
            if display
                customDraw();
            end
    end
    function customDraw()
            clf
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
            pause(0.01)
    end
    function customPhys(id)
        controlVel = ENV.agents(id).velocityControl;
        ENV.agents(id).velocity = controlVel;
        ENV.agents(id).pose = ENV.agents(id).pose + ENV.agents(id).velocity*timeStep;
    end
    function numCol = checkCollide()
        numCol = 0;
        for ii = 1:length(ENV.agents)
            for jj = 1:length(ENV.agents)
                dist = norm(ENV.agents(ii).pose - ENV.agents(jj).pose);
                if dist < agentRadius && ii~= jj
                    numCol = numCol+1;
                end
            end
        end
        numCol = numCol-1;
    end
end


