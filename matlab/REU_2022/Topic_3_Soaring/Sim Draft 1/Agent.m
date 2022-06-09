% Agent class
classdef Agent < handle
    properties
        position = [0.0, 0.0, 0.0]      %m, [x,y,z]
        heading = 0.0                   %rad
        bankAngle = 0.0                 %rad
        velocity = [0.0, 0.0]           %m/s, rad/s, [forward,omega]
        fov = 2*pi                      %rad
    end
    
    methods
        function obj = update(obj,localAgents,thermalStrength)
            % get location of centroid
            % get nearest agent inside no-no zone
            % Calculate desired heading
            % Calculate desired speed
            % Calculate vertical speed
            % Calculate new position
        end
    end
end