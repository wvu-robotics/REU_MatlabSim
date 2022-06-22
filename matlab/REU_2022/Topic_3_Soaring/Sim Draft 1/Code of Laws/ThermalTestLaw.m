classdef ThermalTestLaw
    properties
        % Simulation constraints
        dt = 0.1        %s
        totalTime = 60  %s
        mapSize = [-100,100]   %m, bounds of square map
        %numAgents = 100  %agents
        numThermals = 1 %thermals
        
        % Thermal constraints
        thermalSpeedMin = 5         % m/s
        thermalSpeedMax = 20        % m/s
        thermalRadiusMin = 90       % m
        thermalRadiusMax = 90       % m
        thermalStrengthMin = 50     % m/s, peak updraft speed
        thermalStrengthMax = 100    % m/s, peak updraft speed
        thermalFadeRate = 5         % m/s, rate at which thermals fade in or out 
        thermalMinPlateauTime = 75  % steps at the min strength
        thermalMaxPlateauTime = 100 % steps at the max strength

    end
end