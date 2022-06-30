classdef MaxsLaw
    properties
        %% Variables to be Changed Here
        % Simulation
        totalTime = 1800;           % s
        fpsMult = 15;               % x Real Time
        numThermals = 6;            % thermals
        neighborRadius = 1000;      % m
        neighborAngleRange = 2*pi; %rad
        k = 5;                      % number of nearest neighbors
        frameSkip = 5;              % render every nth frame
        forwardInertia = 10;        %
        bankMin = -2*pi/12;         % rad
        bankMax = 2*pi/12;          % rad
        bankInertia = 1;            %
        fov = 2*pi;                 % rad
        
        % Visuals
        showArrow = true;          %
        renderScale = [300;300];    % [scaleX; scaleY];
        showNeighbors = true;       %
        showFixedRadius = true;     %
        showRange = true;           %

        % Functions to use
        funcName_agentControl     = "agentControl_KNN";
        funcName_findNeighborhood = "findNeighborhood_KNN";

        % Thermal constraints
        thermalSpeedMin = 5         % m/s
        thermalSpeedMax = 20        % m/s
        thermalRadiusMin = 600      % m
        thermalRadiusMax = 1300     % m
        thermalStrengthMin = 50     % m/s, peak updraft speed
        thermalStrengthMax = 100    % m/s, peak updraft speed
        thermalFadeRate = 0.002     % m/s, rate at which thermals fade in or out 
        thermalMinPlateauTime = 600 % steps at the min strength
        thermalMaxPlateauTime = 1000% steps at the max strength

        %% Variables that get changed in the Excel Doc
        separation = 1;
        cohesion   = 1;
        alignment  = 1;
        migration  = 1e-21;
        % these two are kind of the same
        heightPriority = 5;         % For agentControl_Update
        cohesionHeightMult = 5;     % For agentControl_KNN 
        % these two are not at all the same
        heightIgnore = 0.2;         % For agentControl_Update
        separationHeightGap = 2;    % For agentControl_KNN
        
        dt = .1;                    % s
        waggle = pi/48;             % Radians of bank
        waggleTime = 1;             % Seconds of waggle bank        
        numAgents = 40;             % agents

        %% Not to Change
        % Initial conditions
        mapSize = [-4000,4000];     % m, bounds of square map
        agentSpawnPosRange = [-3000,-3000; 3000,3000];  % m, [xMin,yMin;xMax,yMax]
        agentSpawnAltiRange = [1600,1600];              % m, [Min,Max]
        agentSpawnVelRange = [8,0;13,0];                % m/s,rad/s [forwardMin,omegaMin;forwardMax,omegaMax];
        g = 9.81;   
        
        % Visuals
        agentShape_triangle = [-0.5,0.5,-0.5; -0.375,0,0.375];
        agentShape_plane = [-0.5,-0.3,0,0.1,0.2,0.3,0.5,0.3,0.2,0.1,0,-0.3,-0.5;-0.2,-0.1,-0.1,-0.5,-0.5,-0.1,0,0.1,0.5,0.5,0.1,0.1,0.2];
        Arrow = [2 1.5 1.5 0 0 1.5 1.5; 0 .5 .1 .1 -.1 -.1 -.5];
        ThermPatch = [0.8660, 0.5000, 0.0000, -0.5000, -0.8660, -1.0000, -0.8660, -0.5000, -0.0000,  0.5000,  0.8660,  1.0000;
                      0.5000, 0.8660, 1.0000,  0.8660,  0.5000,  0.0000, -0.5000, -0.8660, -1.0000, -0.8660, -0.5000, -0.0000];

        % Thermal constraints
        CMColors = [6 42 127; 41 76 247; 102 59 231; 162 41 216; 222 24 200; 255 192 203] / 255;
        thermalPixels = 50
        
        % Agent Constants
        agentCeiling   = 2600;      % m
        agentFloor     = 0;         % m
        forwardSpeedMin = 5;        % m/s
        forwardSpeedMax = 20;       % m/s
        Sink_A = -0.01843;          %
        Sink_B = 0.3782;            %
        Sink_C = -2.3782;           %


    end

    methods % temporary, remove later
        function strength = getTempThermalStrength(~,agent)
            position = agent.position;
            radius = 600;
            peakStrength = 20;
            thermalPos = [-1000,1000];
            
            dist = norm(position(1:2)-thermalPos);
            closeStrength = peakStrength*(1-(dist/radius)^2);
            strength = max(0,closeStrength);
        end
    end
end

