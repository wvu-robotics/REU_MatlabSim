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
    end
    methods
        % Constructor for Thermal
        function thermal = Thermal(pos, vel, rad, maxStr, curStr)
            if nargin == 0 
                return;
            end
            thermal.position = pos;
            thermal.velocity = vel;
            thermal.radius = rad;
            thermal.maxStrength = maxStr;
            thermal.curStrength = curStr;
        end
    end
end