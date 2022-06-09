% Simulation parameter class
classdef SimLaw
    properties (Constant)
        % Simulation constraints
        dt = 0.1        %s
        totalTime = 60  %s
        mapSize = [-100,100]   %m, bounds of square map
        numAgents = 50  %agents
        numThermals = 2 %thermals
        
        % Initial conditions
        agentSpawnPosRange = [-70,-70; 70,70];     %m, [xMin,yMin;xMax,yMax]
        agentSpawnVelRange = [8,0;13,0];           %m/s,rad/s [forwardMin,omegaMin;forwardMax,omegaMax];
        
        % Rule constraints
        separation = 1.0
        cohesion = 1.0
        alignment = 1.0
        migration = 1.0
        
        % Agent constraints
        neighborRadius = 20     %m
        forwardSpeedMin = 5     %m/s
        forwardSpeedMax = 18    %m/s
        forwardInertia = 10
        bankMin = -pi          %rad/s
        bankMax = pi           %rad/s
        bankInertia = 1
        fov = 2*pi              %rad
        
        % Thermal constraints
        thermalSpeedMin = 0.5   %m/s
        thermalSpeedMax = 2.0   %m/s
        thermalSizeMin = 10     %m
        thermalSizeMax = 50     %m
        thermalStrengthMin = 3  %m/s, peak updraft speed
        thermalStrengthMax = 10 %m/s, peak updraft speed
    end
    
    methods (Static)
        function steps = getSteps()
            steps = SimLaw.totalTime/SimLaw.dt;
        end
    end
end