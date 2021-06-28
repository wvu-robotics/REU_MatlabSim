%% Comparison Test

%Description: Runs three different simulations, one with ORCAController,
%the other with ovalORCAController and feed, and returns the number of time
%steps each took in the same scenario.

%Parameters:
%   initPositions: An Nx2 double where the ith agent starts at
%       initPositions(i,:).

%Returns:
%   
function [ORCAGoalDist, ORCATravTimes, ovalGoalDist, ovalTravTimes, accelGoalDist, accelTravTimes] ...
            = comparisonTest(initPositions, goalPath, agentRadius, timeStep, ...
                             maxTime, idealSpeed, maxSpeed, transCost)
    %Initializes numTimeSteps, numCollisions, and numberOfAgents
    numTimeSteps(2) = 0;
    numCollisions(2) = 0;
    numberOfAgents = size(initPositions,1);

    %Unmodified OCRA Simulation Loop
    agentPositions = initPositions;
    agentVelocities = zeros(numberOfAgents,2);

    for t = 0:timeStep:maxTime
        numTimeSteps(1) = numTimeSteps(1) + 1;
        
        %Computes collision free ORCAVelocities that are 'closest' to the
        %idealVelocities.
        idealVelocities = (goalLocations - agentPositions)./vecnorm(goalLocations - agentPositions, 2, 2) * idealSpeed;
        ORCAVelocites = ORCAController(agentPositions, agentVelocities, idealVelocities, timeHorizon, sensingRange, agentRadius, maxSpeed, velocityDiscritisation, vOptIsZero, responsibility);

        %Computes the acceleration to the ORCAVelocities.
        accelInputs = ORCAVelocites - agentVelocities;
        for i = 1:numberOfAgents
            if norm(accelInputs(i,:)) > 0
                accelInputs(i,:) = accelInputs(i,:) ./ norm(accelInputs(i,:));
            end
        end

        %Applies the accelerations to the current velocities and caps velocity
        agentVelocities = agentVelocities + accelConstant * accelInputs * timeStep;

        %Updates positions & handles collisions
        agentPositions = agentPositions + agentVelocities * timeStep;
        [agentPositions, agentVelocities, newCollisions] = Collider(agentPositions, agentVelocities, agentRadius);
        numCollisions(1) = numCollisions(1) + newCollisions;
        
        %Breaks simulation loop if all robots are at their goals
        if max(vecnorm(agentPositions - goalLocations,2,2)) < 0.2
            break;
        end
    end

    %Modified OCRA Simulation Loop
    agentPositions = initPositions;
    agentVelocities = zeros(numberOfAgents,2);

    for t = 0:timeStep:maxTime
        numTimeSteps(2) = numTimeSteps(2) + 1;
        
        %Computes collision free ORCAVelocities that are 'closest' to the
        %idealVelocities.
        idealVelocities = (goalLocations - agentPositions)./vecnorm(goalLocations - agentPositions, 2, 2) * idealSpeed;
        ORCAVelocites = modifiedORCAController(agentPositions, agentVelocities, idealVelocities, timeHorizon, sensingRange, agentRadius, maxSpeed, velocityDiscritisation, vOptIsZero, responsibility);

        %Computes the acceleration to the ORCAVelocities.
        accelInputs = ORCAVelocites - agentVelocities;
        for i = 1:numberOfAgents
            if norm(accelInputs(i,:)) > 0
                accelInputs(i,:) = accelInputs(i,:) ./ norm(accelInputs(i,:));
            end
        end

        %Applies the accelerations to the current velocities and caps velocity
        agentVelocities = agentVelocities + accelConstant * accelInputs * timeStep;

        %Updates positions & handles collisions
        agentPositions = agentPositions + agentVelocities * timeStep;
        [agentPositions, agentVelocities, newCollisions] = Collider(agentPositions, agentVelocities, agentRadius);
        numCollisions(2) = numCollisions(2) + newCollisions;

        %Breaks simulation loop if all robots are at their goals
        if max(vecnorm(agentPositions - goalLocations,2,2)) < 0.2
            break;
        end
    end
end

