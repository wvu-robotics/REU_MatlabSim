classdef AdamsLaw < handle
    properties
        %% Variables to save
        %% Simulation constraints
        dt = 0.1;        %s
        totalTime = 1200;  %s
        fpsMult = 10;
        frameSkip = 5;
        numAgents = 20;
        numThermals = 20
        
        %% Physical Sim Space
        mapSize = [-3000,3000];     % m, bounds of square map
        agentCeiling   = 2600;    %m
        agentFloor     = 0;      %m
        
        %% Initial conditions
        agentSpawnPosRange = [-1000,-1000; 1000,1000];  % m, [xMin,yMin;xMax,yMax]
        agentSpawnAltiRange = [1600,1600];              % m, [Min,Max]
        agentSpawnVelRange = [8,0;13,0];                % m/s,rad/s [forwardMin,omegaMin;forwardMax,omegaMax];
        g = 9.81;                                  % m/s/s

        %% Rule Parameters
        %% Separation
        separation = 1E5;
        
        separationHeightWidth = 100;
        %m, the current agent will try to separate from other agents that
        %are within separationHeightWidth/2 of <current agent height>
        
        %% Alignment
        alignment = 1E3;
        
        alignmentHeightWidth = 100;
        %m, the current agent will try to align with other agents that
        %are within alignmentHeightWidth/2 of <current agent height>
        
        %% Cohesion
        cohesion = 0.003;
        
        cohesionHeightIgnore = -3;
        %m, other agents below <current agent height + cohesionHeightIgnore> are ignored
        
        cohesionHeightMult = 10;
        %Agents at <current agent height + neighborRadius> are weighted
        %this many times more than an agent at <current agent height + cohesionHeightIgnore>
        
        cohesionAscensionIgnore = 0.5;
        %m/s, agents with relative ascensions less than this are ignored
        
        cohesionAscensionMax = 4; %m/s
        
        cohesionAscensionMult = 50;
        %Agents with relative ascensions of <cohesionAscensionMax> or
        %greater are weighted this many times more than an agent with
        %relative ascension of <cohesionAscensionIgnore>
        
        %% Migration
        migration = 2e-4;
        
        %% Waggle
        waggle = 0.5;
        waggleDurationRange = [0.3,0.5]; %s
        
        %% Agent constraints
        neighborRadius = 1000;     %m
        k = 5; %k-nearest neighbors
        forwardSpeedMin = 5;     %m/s
        forwardSpeedMax = 15;    %m/s
        bankMin = -5*pi/12;           %rad
        bankMax = 5*pi/12;            %rad
        fov = 11/6*pi;              %rad
        Sink_A = -0.01843;
        Sink_B = 0.3782;
        Sink_C = -2.3782;

        %% Visuals
        agentShape_triangle = [-0.5,0.5,-0.5; -0.375,0,0.375];
        agentShape_plane = [-0.5,-0.3,0,0.1,0.2,0.3,0.5,0.3,0.2,0.1,0,-0.3,-0.5;-0.2,-0.1,-0.1,-0.5,-0.5,-0.1,0,0.1,0.5,0.5,0.1,0.1,0.2];
        Arrow = [2 1.5 1.5 0 0 1.5 1.5; 0 .5 .2 .2 -.2 -.2 -.5];
        renderScale = [150;150]; %[scaleX; scaleY];
        showNeighbors = true;
        showFixedRadius = true;
        showRange = false;
        showText = true;
        showArrow = true;
        followAgent = false;
        
        %% Thermal constraints
        CMColors = [6 42 127; 41 76 247; 102 59 231; 162 41 216; 222 24 200; 255 192 203] / 255;
        thermalPixels = 200
        thermalSpawnAttempts = 50; %Number of tries per thermal
        
        thermalSpeedMin = 0         % m/s
        thermalSpeedMax = 0        % m/s
        thermalRadiusMin = 500       % m
        thermalRadiusMax = 800       % m
        thermalStrengthMin = 8     % m/s, peak updraft speed
        thermalStrengthMax = 8    % m/s, peak updraft speed
        thermalFadeRate = 0.008         % m/s, rate at which thermals fade in or out 
        thermalMinPlateauTime = 2000  % steps at the min strength
        thermalMaxPlateauTime = 4000 % steps at the max strength

        %% Functions to use
        funcName_agentControl = "agentControl_Adam";
        funcName_findNeighborhood = "findNeighborhood_fixedRadius";
    end
end

