% Agent class
classdef Agent < handle
    properties
        simLaw
        
        position = [0.0, 0.0, 0.0]      %m, [x,y,z]
        heading = 0.0                   %rad
        bankAngle = 0.0                 %rad
        velocity = [0.0, 0.0, 0.0]           %m/s, rad/s, [forward,omega]
        patchObj = NaN

        isAlive  = true
        markedForDeath = false
        accelDir = 0.0;
        rulesDir = [0.0, 0.0, 0.0, 0.0, 0.0] % S, C, A, M
        rulesMag = [0.0, 0.0, 0.0, 0.0, 0.0] % S, C, A, M
        vsink    = 0                    % m/s
        clearance = 0
        neighbors
        neighborData

        lastWaggle = 0;
        waggleSign = 0;
        
        savedPosition = [0.0, 0.0, 0.0]
        savedVelocity = [0.0, 0.0, 0.0]
        savedHeading = 0.0
    end
    
    methods
        % Render Function (1/2)
        function obj = render(obj,swarm)
            SL = obj.simLaw;

            %% Calculate Agent Orientation Properties
            rotationMatrix = [cos(obj.heading), -sin(obj.heading); sin(obj.heading), cos(obj.heading)];
            shape = SL.agentShape_plane .* SL.renderScale;
            %shape = SL.agentShape_amogus .* SL.renderScale;
            rotatedShape = rotationMatrix * shape; %[x;y] matrix
            rotatedShape = rotatedShape'; %Convert to [x,y];
            globalShape = rotatedShape + obj.position(1:2); %[x,y] matrix
            scaledAlti = 0.8*((obj.position(3)-SL.agentFloor)/(SL.agentCeiling - SL.agentFloor));
            scaledVSpeed = 0.6*(obj.velocity(3)+1)/(1+SL.thermalStrengthMax);
            % Fix altitude
            if scaledAlti < 0
                scaledAlti = 0;
            elseif scaledAlti > 0.8
                scaledAlti = 0.8;
            end
            if scaledVSpeed < 0
                scaledVSpeed = 0;
            elseif scaledVSpeed > 0.8
                scaledVSpeed = 0.8;
            end
            if SL.followAgent && abs(obj.position(3) - swarm.heroAgent.position(3)) > SL.separationHeightWidth/2
                color = hsv2rgb([scaledAlti,0.5,0.5]);
            else
                color = hsv2rgb([scaledAlti,1,1]);
            end
            edgeColor = hsv2rgb([scaledVSpeed,1,1]);
            %fprintf("hue: %g\n",color(1));
            
            %% Do Patch Functions
            if(class(obj.patchObj) == "double")
                obj.patchObj = patch('FaceColor',color);
            end
            obj.patchObj.FaceColor = color;
            obj.patchObj.EdgeColor = edgeColor;
            obj.patchObj.EdgeAlpha = min(1,2*scaledVSpeed);
            obj.patchObj.XData = globalShape(:,1);
            obj.patchObj.YData = globalShape(:,2);
            if ~obj.isAlive
                obj.patchObj.Visible = 'off';
            else
                obj.patchObj.Visible = 'on';
            end

        end
        
        % Save Function (2/2)
        function obj = saveData(obj)
            obj.savedPosition = obj.position;
            obj.savedVelocity = obj.velocity;
            obj.savedHeading = obj.heading;
        end
    end
end