classdef AdamsLaw
    properties
        %% Variables to save
        % Simulation constraints
        dt = 0.1;        %s
        totalTime = 20;  %s
        fpsMult = 1;
        numAgents = 10;
        numThermals = 2
        
        % Physical Sim Space
        mapSize = [-2000,2000];     % m, bounds of square map
        agentCeiling   = 2600;    %m
        agentFloor     = 0;      %m
        
        % Initial conditions
        agentSpawnPosRange = [-1000,-1000; 1000,1000];  % m, [xMin,yMin;xMax,yMax]
        agentSpawnAltiRange = [1500,1700];              % m, [Min,Max]
        agentSpawnVelRange = [8,0;13,0];                % m/s,rad/s [forwardMin,omegaMin;forwardMax,omegaMax];
        g = 9.81;                                  % m/s/s

        % Rule Parameters
        % Basic Boids
        separation = 7;
        cohesion = 0.008;
        alignment = 3;
        
        % Extra Boids
        migration = 2e-13;
        waggle = 0.5;
        
        % Height-Dependent
        separationHeightGap = 10;
        cohesionHeightMult = 8;
        
        % Agent constraints
        neighborRadius = 2000;     %m
        k = 5; %k-nearest neighbors
        forwardSpeedMin = 5;     %m/s
        forwardSpeedMax = 20;    %m/s
        forwardInertia = 10;
        bankMin = -5*pi/12;           %rad
        bankMax = 5*pi/12;            %rad
        bankInertia = 1;
        fov = 3/2*pi;              %rad
        Sink_A = -0.01843;
        Sink_B = 0.3782;
        Sink_C = -2.3782;

        %Visuals
        agentShape_triangle = [-0.5,0.5,-0.5; -0.375,0,0.375];
        agentShape_plane = [-0.5,-0.3,0,0.1,0.2,0.3,0.5,0.3,0.2,0.1,0,-0.3,-0.5;-0.2,-0.1,-0.1,-0.5,-0.5,-0.1,0,0.1,0.5,0.5,0.1,0.1,0.2];
        Arrow = [2 1.5 1.5 0 0 1.5 1.5; 0 .5 .2 .2 -.2 -.2 -.5];
        showArrow = false;
        renderScale = [150;150]; %[scaleX; scaleY];
        showNeighbors = false;
        showFixedRadius = false;
        showRange = false;
        
        % Thermal constraints
        CMColors = [6 42 127; 41 76 247; 102 59 231; 162 41 216; 222 24 200; 255 192 203] / 255;
        thermalPixels = 100
        
        thermalSpeedMin = 0         % m/s
        thermalSpeedMax = 0        % m/s
        thermalRadiusMin = 600       % m
        thermalRadiusMax = 1300       % m
        thermalStrengthMin = 5     % m/s, peak updraft speed
        thermalStrengthMax = 10    % m/s, peak updraft speed
        thermalFadeRate = 0.002         % m/s, rate at which thermals fade in or out 
        thermalMinPlateauTime = 200  % steps at the min strength
        thermalMaxPlateauTime = 400 % steps at the max strength

        % Functions to use
        funcName_agentControl = "agentControl_Adam";
        funcName_findNeighborhood = "findNeighborhood_fixedRadius";
    end
end

