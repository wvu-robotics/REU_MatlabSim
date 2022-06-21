classdef MaxsLaw
    properties
        %% Variables to save
        % Simulation constraints
        dt = .25;           %s
        totalTime = 1800;  %s
        fpsMult = 30;
        mapSize = [-4000,4000];   %m, bounds of square map
        numAgents = 40;  %agents
        numThermals = 4; %thermals

        % Initial conditions
        agentSpawnPosRange = [-3000,-3000; 3000,3000];     %m, [xMin,yMin;xMax,yMax]
        agentSpawnAltiRange = [1600,1600];             %m, [Min,Max]
        agentSpawnVelRange = [8,0;13,0];           %m/s,rad/s [forwardMin,omegaMin;forwardMax,omegaMax];
        g = 9.81;                                  % m/s/s

        % Rule constraints
        separation = 1;
        cohesion   = 1;
        alignment  = 1;
        migration  = 1e-21;
        waggle = 0;
        heightPriority = 5;
        heightIgnore = -0.2;

        % Agent constraints
        neighborRadius = 1000;     %m
        agentCeiling   = 2600;    %m
        agentFloor     = 0;      %m
        forwardSpeedMin = 5;     %m/s
        forwardSpeedMax = 20;    %m/s
        forwardInertia = 10;
        bankMin = -2*pi/12;           %rad
        bankMax = 2*pi/12;            %rad
        bankInertia = 1;
        fov = 2*pi;              %rad
        Sink_A = -0.01843;
        Sink_B = 0.3782;
        Sink_C = -2.3782;

        % Thermal constraints
        tempThermalStrength = 10;
        thermalSpeedMin = 20;    % m/s
        thermalSpeedMax = 50;    % m/s
        thermalRadiusMin = 5;    % m
        thermalRadiusMax = 20;   % m
        thermalStrengthMin = 0;  % m/s, peak updraft speed
        thermalStrengthMax = 10; % m/s, peak updraft speed
        thermalFadeRate = 1;     % m/s, rate at which thermals fade in or out 
        thermalPlateauTime = 5;  % steps at a max or min strength

        %Visuals
        agentShape_triangle = [-0.5,0.5,-0.5; -0.375,0,0.375];
        agentShape_plane = [-0.5,-0.3,0,0.1,0.2,0.3,0.5,0.3,0.2,0.1,0,-0.3,-0.5;-0.2,-0.1,-0.1,-0.5,-0.5,-0.1,0,0.1,0.5,0.5,0.1,0.1,0.2];
        Arrow = [2 1.5 1.5 0 0 1.5 1.5; 0 .5 .1 .1 -.1 -.1 -.5];
        ThermPatch = [0.8660, 0.5000, 0.0000, -0.5000, -0.8660, -1.0000, -0.8660, -0.5000, -0.0000,  0.5000,  0.8660,  1.0000;
                      0.5000, 0.8660, 1.0000,  0.8660,  0.5000,  0.0000, -0.5000, -0.8660, -1.0000, -0.8660, -0.5000, -0.0000];
        showArrow = false;
        renderScale = [300;300]; %[scaleX; scaleY];

        % Functions to use
        funcName_agentControl = "agentControl_Update";
        funcName_findNeighborhood = "findNeighborhood_fixedRadius";
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

