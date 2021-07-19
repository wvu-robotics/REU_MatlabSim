%% comparisonTest: Comparison Tester
%
% Description: Given a goalPath for the team of robots, runs two simulations
% where different controllers move the robots. One is the original
% ORCAController, and the other is the accelerationController. It gathers
% and returns data about the different simulations.
% The times hold the number of time steps taken between goal positions.
% The distances hold the distance traveled to get between waypoints.
% The smoothness holds the amount of turning that an agent has done in rad
% divided by the time that that agent took.
%
% Parameters:
%   goalPath: An Nx2xP double where the ith waypoint that the jth agent has
%       to get to is goalPath(j,:,i). Agents must reach the ith waypoint
%       before proceeding to the i+1st waypoint, and must stay put once
%       they've reached their last waypoint. Ideally,
%       -10 < goalPath(i,j,k) < 10.
%   initPositions: An Nx2 double where the ith agent starts at
%       initPositions(i,:). Ideally, -10 < initPositions(i,j) < 10.
%   timeStep: A positive double for the amount of time between frames in
%       the sims
%   realTime: A boolean that determines whether the sim should or shouldn't
%       run at an accurate rate given the timeStep.
%   mapSize: A positive double where the environment size will be -mapSize
%       to mapSize in the both x and y axes.
%   maxSpeed: A positive double for how fast an agent can go
%   maxSpeed: A positive double for how fast an agent wants to go
%   measuringRange: A positive double where an agent can only see other
%       agents that are a maximum distance of measuringRange away
%
% Returns:
%   ORCATimes: An NxP double where the number of time steps that the ith
%       ORCA controlled agent took to get to from its j-1st waypoint to its
%       jth waypoint is ORCATimes(i,j). ORCATimes(i,1) is the time the ith
%       agent took to get from its initial position to its first waypoint.
%   ORCADistances: An NxP double where ORCADistances(i,j) is the distance
%       the ith agent traveled to get from its j-1th waypoint to its jth
%       waypoint. ORCADistances(i,1) is the distance the ith agent traveled
%       to get to its 1st waypoint.
%   ORCASmoothness: An NxP double where ORCASmoothness(i,j) is the
%       smoothness of agent i's path on its way to the jth waypoint.
%   accelTimes: An NxP double where the number of time steps that the ith
%       ORCA controlled agent took to get to from its j-1st waypoint to its
%       jth waypoint is ORCATimes(i,j). ORCATimes(i,1) is the time the ith
%       agent took to get from its initial position to its first waypoint.
%   accelDistances: An NxP double where accelDistances(i,j) is the distance
%       the ith agent traveled to get from its j-1th waypoint to its jth
%       waypoint. accelDistances(i,1) is the distance the ith agent
%       traveled to get to its 1st waypoint.
%   accelSmoothness: An NxP double where accelSmoothness(i,j) is the
%       smoothness of agent i's path on its way to the jth waypoint.
function [ORCATimes, ORCADistances, ORCASmoothness, accelTimes, accelDistances, accelSmoothness] ...
    = comparisonTest(goalPath, initPositions, agentRadius, timeStep, realTime, mapSize, maxSpeed, idealSpeed, measuringRange)
    
    numberOfAgents = size(initPositions,1);
    pathLength = size(goalPath,3);
    
    % == ORCA Simulation ================================================ %
    
    %Preallocates the ORCA statistics
    ORCATimes = zeros(numberOfAgents,pathLength);
    ORCADistances = zeros(numberOfAgents,pathLength);
    ORCASmoothness = zeros(numberOfAgents,pathLength);
    
    %Creates a variable to hold the current goals for each agent
    goalLocations = goalPath(:,:,1);
    
    %Creates the environment for testing ORCA
    ENV = agentEnv(numberOfAgents, @ORCAController, mapSize, timeStep);

    for i = 1:numberOfAgents
        ENV.agents(i).maxSpeed = maxSpeed;
        ENV.agents(i).idealSpeed = idealSpeed;
        ENV.agents(i).measuringRange = measuringRange;
        ENV.agents(i).setShape(circle(agentRadius));
        ENV.setAgentColor(i,[0,0,1]);
        ENV.updateCollisionList('A',i);
    end

    ENV.setAgentPositions(initPositions);
    ENV.setGoalPositions(goalLocations);
    ENV.realTime = realTime;
    ENV.pathVisibility(false);
    ENV.collisionsOn(false);
    ENV.updateEnv; %required after each agent is finally initialized
    
    %Keeps track of how far along each agent is. If agent i must go to its
    %jth waypoint, then pathCounters(i) = j. If agent i has reached all of
    %its waypoints, then pathCounters(i) = pathLength + 1.
    pathCounters = ones(numberOfAgents,1);
    
    %While some agents haven't made it to their last waypoint
    while min(pathCounters) <= pathLength
        ENV.tick();

        %Increments the number of time steps each agents took to get to
        %their waypoints. If an agent has reached their last waypoint,
        %their times don't increment.
        for i = 1:numberOfAgents
            %If the agent hasn't finished their journey
            if pathCounters(i) <= pathLength
                ORCATimes(i,pathCounters(i)) = ORCATimes(i,pathCounters(i)) + 1;
            end
        end
        
        %Increments the distance each agent traveled to get to their
        %waypoints. If an agent has reached their last waypoint, their
        %distances don't increment.
        for i = 1:numberOfAgents
            %If the agent hasn't finished their journey
            if pathCounters(i) <= pathLength
                ORCADistances(i,pathCounters(i)) = ORCADistances(i,pathCounters(i)) + norm(ENV.agents(i).velocity) * timeStep;
            end
        end
        
        %Adds to the amount each agent has turned
        %If there are more than three positions on the agents' paths
        if size(ENV.agents(1).path, 1) >= 3
            %Adds to the smoothness of an agent
            for i = 1:numberOfAgents
                %If the agent hasn't finished their journey
                if pathCounters(i) <= pathLength
                    p2 = ENV.agents(i).path(end,:);
                    p1 = ENV.agents(i).path(end-1,:);
                    p0 = ENV.agents(i).path(end-2,:);
                    
                    %Adds to the amount agent i has turned
                    if abs(dot(p2-p1,p1-p0) / (norm(p2-p1) * norm(p1-p0))) <= 1
                        ORCASmoothness(i,pathCounters(i)) = ORCASmoothness(i,pathCounters(i)) + acos(dot(p2-p1,p1-p0) / (norm(p2-p1) * norm(p1-p0)));
                    end
                end
            end
        end

        %Moves an agent's waypoint to the next waypoint on the path once
        %they reach it. If an agent has reached their last waypoint,
        %that waypoint is not changed.
        for i = 1:numberOfAgents
            %If agent i has reached its goal.
            if norm(ENV.agents(i).pose - goalLocations(i,:)) < agentRadius
                %If the agent hasn't finished its journey
                if pathCounters(i) < pathLength
                    goalLocations(i,:) = goalPath(i,:,pathCounters(i)+1);
                    ENV.setGoalPositions(goalLocations);
                end
                %If the agent hasn't finished their journey
                if pathCounters(i) <= pathLength
                    pathCounters(i) = pathCounters(i) + 1;
                end
            end
        end
    end
    
    %Takes the average of the sum that is stored in ORCASmoothness
    ORCASmoothness = ORCASmoothness ./ ORCATimes;
    
    
    
    %Deletes the environment and closes all windows for the next sim 
    clear ENV;
    close all;
    
    % == accel Simulation ================================================ %
    
    %Preallocates the accelTimes and accelDistances
    accelTimes = zeros(numberOfAgents,pathLength);
    accelDistances = zeros(numberOfAgents,pathLength);
    accelSmoothness = zeros(numberOfAgents,pathLength);
    
    %Creates a variable to hold the current goals for each agent
    goalLocations = goalPath(:,:,1);
    
    %Creates the environment for testing accelerationController
    ENV = agentEnv(numberOfAgents, @accelerationController, mapSize, timeStep);

    for i = 1:numberOfAgents
        ENV.agents(i).maxSpeed = maxSpeed;
        ENV.agents(i).idealSpeed = idealSpeed;
        ENV.agents(i).measuringRange = measuringRange;
        ENV.agents(i).setShape(circle(agentRadius));
        ENV.setAgentColor(i,[0,0,1]);
        ENV.updateCollisionList('A',i);
    end

    ENV.setAgentPositions(initPositions);
    ENV.setGoalPositions(goalLocations);
    ENV.realTime = realTime;
    ENV.pathVisibility(false);
    ENV.collisionsOn(false);
    ENV.updateEnv; %required after each agent is finally initialized
    
    %Keeps track of how far along each agent is. If agent i must go to its
    %jth waypoint, then pathCounters(i) = j. If agent i has reached all of
    %its waypoints, then pathCounters(i) = pathLength + 1.
    pathCounters = ones(numberOfAgents,1);
    
    %While some agents haven't made it to their last waypoint
    while min(pathCounters) <= pathLength
        ENV.tick();

        %Increments the number of time steps each agents took to get to
        %their waypoints. If an agent has reached their last waypoint,
        %their times don't increment.
        for i = 1:numberOfAgents
            %If the agent hasn't finished their journey
            if pathCounters(i) <= pathLength
                accelTimes(i,pathCounters(i)) = accelTimes(i,pathCounters(i)) + 1;
            end
        end
        
        %Increments the distance each agent traveled to get to their
        %waypoints. If an agent has reached their last waypoint, their
        %distances don't increment.
        for i = 1:numberOfAgents
            %If the agent hasn't finished their journey
            if pathCounters(i) <= pathLength
                accelDistances(i,pathCounters(i)) = accelDistances(i,pathCounters(i)) + norm(ENV.agents(i).velocity) * timeStep;
            end
        end
        
        %Adds to the amount each agent has turned
        %If there are more than three positions on the agents' paths
        if size(ENV.agents(1).path, 1) >= 3
            %Adds to the smoothness of an agent
            for i = 1:numberOfAgents
                %If the agent hasn't finished their journey
                if pathCounters(i) <= pathLength
                    p2 = ENV.agents(i).path(end,:);
                    p1 = ENV.agents(i).path(end-1,:);
                    p0 = ENV.agents(i).path(end-2,:);
                    
                    %Adds to the amount agent i has turned
                    if abs(dot(p2-p1,p1-p0) / (norm(p2-p1) * norm(p1-p0))) <= 1
                        accelSmoothness(i,pathCounters(i)) = accelSmoothness(i,pathCounters(i)) + acos(dot(p2-p1,p1-p0) / (norm(p2-p1) * norm(p1-p0)));
                    end
                end
            end
        end

        %Moves an agent's waypoint to the next waypoint on the path once
        %they reach it. If an agent has reached their last waypoint,
        %that waypoint is not changed.
        for i = 1:numberOfAgents
            %If agent i has reached its goal.
            if norm(ENV.agents(i).pose - goalLocations(i,:)) < agentRadius
                %If the agent hasn't finished its journey
                if pathCounters(i) < pathLength
                    goalLocations(i,:) = goalPath(i,:,pathCounters(i)+1);
                    ENV.setGoalPositions(goalLocations);
                end
                %If the agent hasn't finished their journey
                if pathCounters(i) <= pathLength
                    pathCounters(i) = pathCounters(i) + 1;
                end
            end
        end
    end
    
    %Takes the average of the sum that is stored in ORCASmoothness
    accelSmoothness = accelSmoothness ./ accelTimes;
    
    %Deletes the environment and closes all windows
    clear ENV;
    close all;
end

