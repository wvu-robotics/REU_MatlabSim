% Thermal class
classdef Thermal < handle
    properties
        position = [0 0];
        velocity = [0 0];
        size = 0;
        strength = 0;
    end
    methods
        % Constructor for Thermal
        function thermal = Thermal(pos, vel, sz, str)
            if nargin == 0 
                return;
            end
            thermal.position = pos;
            thermal.velocity = vel;
            thermal.size = sz;
            thermal.strength = str;
        end
    end
end