% Thermal class
classdef Thermal < handle
    properties
        position = [0 0];
        velocity = [0 0];
        radius = 0;
        maxStrength = 0;
        curStrength = 0;
        stepCount = 0;
        strengthDirection = 1;
        bounds = [0 0];
    end
    methods
        % Default constructor for Thermal
        function thermal = Thermal(simLaw)
            thermalMap.simLaw = simLaw;
            SL = thermalMap.simLaw;

            % Randomize the thermal's properties
            thermal.position(1) = round(Utility.randIR(SL.mapSize(1) + SL.thermalRadiusMax,SL.mapSize(2) - SL.thermalRadiusMax));
            thermal.position(2) = round(Utility.randIR(SL.mapSize(1) + SL.thermalRadiusMax,SL.mapSize(2) - SL.thermalRadiusMax));
                    
            % Randomly decide if the velocity is negative or positive
            randFactor = randi([0 1],1,2);
            randFactor(randFactor == 0) = -1;
            thermal.velocity(1) = Utility.randIR(SL.thermalSpeedMin,SL.thermalSpeedMax)*randFactor(1);
            thermal.velocity(2) = Utility.randIR(SL.thermalSpeedMin,SL.thermalSpeedMax)*randFactor(2);
                    
            thermal.radius = round(Utility.randIR(SL.thermalRadiusMin,SL.thermalRadiusMax));
            thermal.maxStrength = round(Utility.randIR(SL.thermalStrengthMin + 1,SL.thermalStrengthMax));
            thermal.curStrength = round(Utility.randIR(1, thermal.maxStrength));
        
            thermal.bounds = [SL.mapSize(1)+thermal.radius+1 SL.mapSize(2)-thermal.radius-1];
        end
    end
end