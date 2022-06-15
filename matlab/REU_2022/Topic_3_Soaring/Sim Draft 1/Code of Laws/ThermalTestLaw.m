classdef ThermalTestLaw
    properties
        % Simulation constraints
        dt = 0.1        %s
        totalTime = 60  %s
        mapSize = [-100,100]   %m, bounds of square map
        numAgents = 100  %agents
        numThermals = 4 %thermals
        
        % Initial conditions
        agentSpawnPosRange = [-70,-70; 70,70];     %m, [xMin,yMin;xMax,yMax]
        agentSpawnAltiRange = [5,50];             %m, [Min,Max]
        agentSpawnVelRange = [8,0;13,0];           %m/s,rad/s [forwardMin,omegaMin;forwardMax,omegaMax];
        g = 9.81;                                  % m/s/s
        
        % Rule constraints
        separation = 5.0
        cohesion = 0.015
        alignment = 5
        migration = 1e-12
        
        % Agent constraints
        neighborRadius = 50     %m
        agentCeiling   = 100    %m
        agentFloor     = 0      %m
        forwardSpeedMin = 5     %m/s
        forwardSpeedMax = 18    %m/s
        forwardInertia = 10
        bankMin = -5*pi/12           %rad
        bankMax = 5*pi/12            %rad
        bankInertia = 1
        fov = 2*pi              %rad
        Sink_A = -0.01843
        Sink_B = 0.3782
        Sink_C = -2.3782
        
        % Thermal constraints
        thermalSpeedMin = 20        % m/s
        thermalSpeedMax = 50        % m/s
        thermalRadiusMin = 10       % m
        thermalRadiusMax = 25       % m
        thermalStrengthMin = 3      % m/s, peak updraft speed
        thermalStrengthMax = 10     % m/s, peak updraft speed
        thermalFadeRate = 0.25      % m/s, rate at which thermals fade in or out 
        thermalMinPlateauTime = 10; % steps at the min strength
        thermalMaxPlateauTime = 50  % steps at the max strength

        %Visuals
        agentShape_triangle = [-0.5,0.5,-0.5; -0.375,0,0.375]
        agentShape_plane = [-0.5,-0.3,0,0.1,0.2,0.3,0.5,0.3,0.2,0.1,0,-0.3,-0.5;-0.2,-0.1,-0.1,-0.5,-0.5,-0.1,0,0.1,0.5,0.5,0.1,0.1,0.2]
        Arrow = [2 1.5 1.5 0 0 1.5 1.5; 0 .5 .2 .2 -.2 -.2 -.5];
        renderScale = [10;10]; %[scaleX; scaleY];
    end
end