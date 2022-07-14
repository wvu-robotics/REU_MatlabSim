classdef ColesLaw
    properties
         %% Variables to be Changed Here
        % Simulation
        totalTime = 5400;           % s
        fpsMult = 10;               % x Real Time
        neighborRadius = 2000;      % m
        k = 10;                     % number of nearest neighbors
        frameSkip = 5;              % render every nth frame
        neighborFrameSkip = 5;      % check the neighbors every nth frame
        forwardInertia = 10;        % <unused>
        bankMin = -2*pi/12;         % rad
        bankMax = 2*pi/12;          % rad
        bankInertia = 1;            % <unused>
        fov = 2 * pi;               % rad - 11 * pi / 6 is more accurate
        forwardSpeedMin = 8;        % m/s
        forwardSpeedMax = 13;       % m/s

        % Data
        collisionCount = 0;

        % For Adams Law
        heightDesireBounds = [1000,2000];
        heightDesireMagBounds = [1,0];
        collisionKillDistance = 4;
        relativeAscensionBounds = [1,4];
        relativeAscensionMagBounds = [0,1];
        relativeHeightMagBounds = [-1,1];
        coh_relativeAscension = [1,2];
        coh_heightDesire = [5,1];
        sep_relativeHeight = [3,1];
        sep_heightDesire = [1,2];
        align_relativeHeight = [3,1];
        align_heightDesire = [1,2];
        mig_heightDesire = [0,1];
        
        % Visuals
        showArrow = false;           %
        followAgent = false;        %
        followRadius = 1000;
        renderScale = [200;200];    % [scaleX; scaleY];
        showNeighbors = true;       %
        showFixedRadius = true;     %
        showRange = false;          %
        showText  = false;          %
        stopWhenDead = true;        %

        % Functions to use
        funcName_agentControl     = "agentControl_Frank";
        funcName_findNeighborhood = "findNeighborhood_KNNInFixedRadius";

        % Thermal constraints
        numThermals = 6;            % thermals
        thermalPixels = 50
        thermalSpeedMin = 0         % m/s
        thermalSpeedMax = 0         % m/s
        thermalRadiusMin = 600      % m
        thermalRadiusMax = 1300     % m
        thermalStrengthMin = 3      % m/s, peak updraft speed
        thermalStrengthMax = 10     % m/s, peak updraft speed
        thermalFadeRate = 0.001     % m/s, rate at which thermals fade in or out 
        thermalMinPlateauTime = 600 % steps at the min strength
        thermalMaxPlateauTime = 1000 % steps at the max strength
        thermalSpawnAttempts = 75;  % number of attempts

        %% Variables that get changed in the Excel Doc
        separation = 10^-2;
        cohesion   = 10^-4;
        alignment  = 10^-2;
        migration  = 1E-21;
        % these two are kind of the same
        heightPriority = 5;         % For agentControl_Update
        cohesionHeightMult = 2;     % For agentControl_KNN 
        % these two are not at all the same
        heightIgnore = 0.2;         % For agentControl_Update
        separationHeightWidth = 2;  % For agentControl_KNN
        
        dt = .2;                    % s
        waggle = 0;                 % Radians of bank
        waggleTime = 0;             % Seconds of waggle bank        
        numAgents = 30;             % agents
        waggleDurationRange = [0.3,0.5]; 

        %% Not to Change
        % Initial conditions
        mapSize = [-4000,4000];     % m, bounds of square map
        agentSpawnPosRange = [-3000,-3000; 3000,3000];  % m, [xMin,yMin;xMax,yMax]
        agentSpawnAltiRange = [2000,2000];              % m, [Min,Max]
        agentSpawnVelRange = [8,0;13,0];                % m/s,rad/s [forwardMin,omegaMin;forwardMax,omegaMax];
        g = 9.81;   
        
        % Visuals
        agentShape_triangle = [-0.5,0.5,-0.5; -0.375,0,0.375];
        agentShape_plane = [-0.5,-0.3,0,0.1,0.2,0.3,0.5,0.3,0.2,0.1,0,-0.3,-0.5;-0.2,-0.1,-0.1,-0.5,-0.5,-0.1,0,0.1,0.5,0.5,0.1,0.1,0.2];
        agentShape_amogus = [.25 .5 0 0 .5 .5 .15 .15 -.15 -.15 -.5 -.5 -.75 -.75 -.25;
                             .75 .4 .4 .15 .15 -1 -1 -.6 -.6 -1 -1 -.55 -.25 .25 .75];
        Arrow = [2 1.5 1.5 0 0 1.5 1.5; 0 .5 .1 .1 -.1 -.1 -.5];
        ThermPatch = [0.8660, 0.5000, 0.0000, -0.5000, -0.8660, -1.0000, -0.8660, -0.5000, -0.0000,  0.5000,  0.8660,  1.0000;
                      0.5000, 0.8660, 1.0000,  0.8660,  0.5000,  0.0000, -0.5000, -0.8660, -1.0000, -0.8660, -0.5000, -0.0000];
        CMColors = [6 42 127; 41 76 247; 102 59 231; 162 41 216; 222 24 200; 255 192 203] / 255;
        
        % Agent Constants
        bankAngleFactor = 0.05;
        agentCeiling   = 2600;      % m
        agentFloor     = 0;         % m
        Sink_A = -0.01843;          %
        Sink_B = 0.3782;            %
        Sink_C = -2.3782;           %
    end
    
%     methods
%         function strength = getTempThermalStrength(~,agent)
%             position = agent.position;
%             radius = 50;
%             peakStrength = 20;
%             thermalPos = [-0,-0];
%             
%             dist = norm(position(1:2)-thermalPos);
%             closeStrength = peakStrength*(1-(dist/radius)^2);
%             strength = max(0,closeStrength);
%         end
%     end
end

