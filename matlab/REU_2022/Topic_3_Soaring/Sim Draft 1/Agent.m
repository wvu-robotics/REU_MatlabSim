% Agent class
classdef Agent < handle
    properties
        position = [0.0, 0.0, 0.0]      %m, [x,y,z]
        heading = 0.0                   %rad
        bankAngle = 0.0                 %rad
        velocity = [0.0, 0.0]           %m/s, rad/s, [forward,omega]
    end
    
    methods
        function obj = update(obj,localAgents,thermalStrength)
            
            for other = 1:numAgents
                %% get location of centroid


                %% get nearest agent inside no-no zone
                % nonozone = SimLaw.neighborRadius;
            end
            
            %% Calculate desired heading

            %% Calculate desired speed
            
            %% Calculate vertical speed
            vsink = (a*obj.velocity(1).^2 + b*obj.velocity(1) + c)...
                    / sqrt(cos(obj.bankAngle*2*pi/180));
            vspeed = vsink + thermalStrength;
            %% Calculate new position

        end
    end
end