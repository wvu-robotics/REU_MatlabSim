% Simulation parameter class
classdef SimLaw
    properties (Constant)
        % Simulation constraints
        dt = 0.1        %s
        totalTime = 60  %s
        mapSize = [-100,100]   %m, bounds of square map
        numAgents = 50  %agents
        numThermals = 4 %thermals
        
        % Initial conditions
        agentSpawnPosRange = [-70,-70; 70,70];     %m, [xMin,yMin;xMax,yMax]
        agentSpawnVelRange = [8,0;13,0];           %m/s,rad/s [forwardMin,omegaMin;forwardMax,omegaMax];
        g = 9.81;                                  % m/s/s
        
        % Rule constraints
        separation = 1.0
        cohesion = 1.0
        alignment = 1.0
        migration = 1e-2
        
        % Agent constraints
        neighborRadius = 20     %m
        forwardSpeedMin = 5     %m/s
        forwardSpeedMax = 18    %m/s
        forwardInertia = 10
        bankMin = -11*pi/12           %rad/s
        bankMax = 11*pi/12            %rad/s
        bankInertia = 1
        fov = 2*pi              %rad
        Sink_A = -0.01843
        Sink_B = 0.3782
        Sink_C = -2.3782
        
        % Thermal constraints
        thermalSpeedMin = 20    % m/s
        thermalSpeedMax = 50    % m/s
        thermalRadiusMin = 5    % m
        thermalRadiusMax = 20   % m
        thermalStrengthMin = 3  % m/s, peak updraft speed
        thermalStrengthMax = 10 % m/s, peak updraft speed
        thermalFadeRate = 1     % m/s, rate at which thermals fade in or out 
        
        %Visuals
        agentShape_triangle = [-0.5,0.5,-0.5; -0.375,0,0.375]
        agentShape_plane = [-0.5,-0.3,0,0.1,0.2,0.3,0.5,0.3,0.2,0.1,0,-0.3,-0.5;-0.2,-0.1,-0.1,-0.5,-0.5,-0.1,0,0.1,0.5,0.5,0.1,0.1,0.2]
        renderScale = [5;5]; %[scaleX, scaleY];
    end
    
    methods (Static)
        function steps = getSteps()
            steps = SimLaw.totalTime/SimLaw.dt;
        end
    end
end