% Agent class
classdef Agent < handle
    properties
        position = [0.0, 0.0, 0.0]      %m, [x,y,z]
        heading = 0.0                   %rad
        bankAngle = 0.0                 %rad
        velocity = [0.0, 0.0]           %m/s, rad/s, [forward,omega]
        patchObj = NaN
        patchArr = NaN
        isAlive  = true
        accelDir = 0.0;
        
        savedPosition = [0.0, 0.0, 0.0]
        savedVelocity = [0.0, 0.0]
        savedHeading = 0.0
    end
    
    methods
        function obj = render(obj)
            rotationMatrix = [cos(obj.heading), -sin(obj.heading); sin(obj.heading), cos(obj.heading)];
            %AccelMatrix    = [cos(obj.accelDir), -sin(obj.accelDir); sin(obj.accelDir), cos(obj.accelDir)];
            shape = SimLaw.agentShape_plane .* SimLaw.renderScale;
            %arrow = SimLaw.Arrow .* SimLaw.renderScale;
            rotatedShape = rotationMatrix * shape; %[x;y] matrix
            rotatedShape = rotatedShape'; %Convert to [x,y];
            %rotatedArrow = AccelMatrix * arrow;
            %rotatedArrow = rotatedArrow';
            globalShape = rotatedShape + obj.position(1:2); %[x,y] matrix
            %globalArrow = rotatedArrow + obj.position(1:2);
            scaledAlti = 0.8*((obj.position(3)-SimLaw.agentFloor)/(SimLaw.agentCeiling - SimLaw.agentFloor));
            color = hsv2rgb([scaledAlti,1,1]);
            %fprintf("hue: %g\n",color(1));
            
            if(class(obj.patchObj) == "double")
                obj.patchObj = patch('FaceColor',color);
                %obj.patchArr = patch('FaceColor',color);
            end
            obj.patchObj.FaceColor = color;
            obj.patchObj.XData = globalShape(:,1);
            obj.patchObj.YData = globalShape(:,2);
            if ~obj.isAlive
                obj.patchObj.Visible = 'off';
            end
            %obj.patchArr.XData = globalArrow(:,1);
            %obj.patchArr.YData = globalArrow(:,2);
        end
        
        function obj = saveData(obj)
            obj.savedPosition = obj.position;
            obj.savedVelocity = obj.velocity;
            obj.savedHeading = obj.heading;
        end
    end
end