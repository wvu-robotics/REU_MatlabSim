%Clears the Workspace, Command Window, and all Figures
clear
clc
close all

%Defines the constants for ...
%   World Building
numberOfAgents = 5;
agentRadius = 1;
mapSize = 10;
timeStep = .05;
maxTime = 80;

%   VO's and ORCA
timeHorizon = 10;
sensingRange = 20;
velocityDiscritisation = 0.05;
vOptIsZero = false;
responsibility = 0.5;

%   Control Constants and Limitations
safetyMargin = 1.1;
idealSpeed = .5;
maxSpeed = 2;
accelConstant = 1;

%Main Comparison Loop
numCompares = 30;
numTimeSteps = zeros(numCompares,2);
numCollisions = zeros(numCompares,2);

initPositions = zeros(numberOfAgents, 2);
goalLocations = zeros(numberOfAgents, 2);

for i = 1:numCompares
    %Initializes random positions and goals around a circle
    for j = 1:numberOfAgents
        theta = rand()*2*pi;
        initPositions(j,:) = [cos(theta),sin(theta)]*mapSize*(.7+(rand()-0.5)*.2);
        goalLocations(j,:) = [cos(theta+pi),sin(theta+pi)]*mapSize*(.7+(rand()-0.5)*.2);
    end
    
    [numTimeSteps(i,:), numCollisions(i,:)] = comparisonTest(initPositions, goalLocations, agentRadius, timeStep, maxTime, ...
                                                             timeHorizon, sensingRange, velocityDiscritisation, vOptIsZero, ...
                                                             responsibility, idealSpeed, maxSpeed, accelConstant);
end