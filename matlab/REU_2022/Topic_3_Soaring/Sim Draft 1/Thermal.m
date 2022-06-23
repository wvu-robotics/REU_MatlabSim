% Thermal class
classdef Thermal < handle
    properties
        radius = 0;
        position = [0 0];
        velocity = [0 0];
        maxStrength = 0;
        curStrength = 0;
        stepCount = 0;
        strengthDirection = 1;
        bounds = [0 0];
    end
    methods
        function thermal = Thermal(simLaw)
            % Randomize the thermal's properties
            thermal.radius = round(Utility.randIR(simLaw.thermalRadiusMin,simLaw.thermalRadiusMax));

            thermal.position(1) = round(Utility.randIR(simLaw.mapSize(1) + thermal.radius + 1, simLaw.mapSize(2) - thermal.radius - 1));
            thermal.position(2) = round(Utility.randIR(simLaw.mapSize(1) + thermal.radius + 1,simLaw.mapSize(2) - thermal.radius - 1));

            % Randomly decide if the velocity is negative or positive
            randFactor = randi([0 1],1,2);
            randFactor(randFactor == 0) = -1;
            thermal.velocity(1) = Utility.randIR(simLaw.thermalSpeedMin,simLaw.thermalSpeedMax)*randFactor(1);
            thermal.velocity(2) = Utility.randIR(simLaw.thermalSpeedMin,simLaw.thermalSpeedMax)*randFactor(2);

            thermal.maxStrength = round(Utility.randIR(simLaw.thermalStrengthMin + 1,simLaw.thermalStrengthMax));
            thermal.curStrength = round(Utility.randIR(1, thermal.maxStrength));

            thermal.bounds = [simLaw.mapSize(1)+thermal.radius+1 simLaw.mapSize(2)-thermal.radius-1];
        end
    end
end