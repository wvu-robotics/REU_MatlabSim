% Thermal class
classdef Thermal < handle
    properties
        position = [0 0];
        velocity = [0 0];
        radius = 0;
        strength = 0;
    end
    methods
        % Constructor for Thermal
        function thermal = Thermal(pos, vel, rad, str)
            if nargin == 0 
                return;
            end
            thermal.position = pos;
            thermal.velocity = vel;
            thermal.radius = rad;
            thermal.strength = str;
        end
    end
end