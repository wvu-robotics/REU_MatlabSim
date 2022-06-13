% Agent class
classdef Agent < handle
    properties
        position = [0.0, 0.0, 0.0]      %m, [x,y,z]
        heading = 0.0                   %rad
        bankAngle = 0.0                 %rad
        velocity = [0.0, 0.0]           %m/s, rad/s, [forward,omega]
        patchObj = NaN
        
    end
    
    methods
        % Function 1 : Updates telemetry of obj
        function obj = update(obj,localAgents,thermalStrength, target)
            %% Get Centroid
            Centroid = obj.position;
            distance = zeros(1,length(localAgents));
            for i = 1:length(localAgents)
                if localAgents(i).position(3) <= 0
                    continue;
                end
                distance(i) = norm(obj.position - localAgents(i).position);
                Centroid = Centroid + localAgents(i).position*Utility.isNear(obj, localAgents(i), NaN);
            end
            
            %% Get Vectors
            % Cohesion
            distToCentroid = norm(obj.position - Centroid);
            if distToCentroid == 0
                centroidUnit = [0,0,0];
            else
                centroidUnit   = (obj.position - Centroid) / distToCentroid;
            end
            
            % Seperation & Alignment
            nearest        = find(distance == min(distance));
            if ~isempty(nearest)
                nearest        = nearest(1);
                distToNearest  = norm(obj.position - localAgents(nearest).position);
                nearestUnit    = (obj.position - localAgents(nearest).position) / distToNearest;
                nearestVelUnit = localAgents(nearest).velocity(1)*...
                                [cos(localAgents(nearest).heading), sin(localAgents(nearest).heading), 0];
            else
                distToNearest  = 1;
                nearestUnit    = [0,0,0];
                nearestVelUnit = [0,0,0];
            end
            
            % Alignment
            

            % Migration
            distToTarget   = norm(obj.position(1:2) - target);
            targetUnit     = (obj.position(1:2) - target) / distToTarget;
            targetUnit     = [targetUnit 0];
            
            %% Get Accel
            accelMag_cohesion   = SimLaw.cohesion   *      distToCentroid^2; %Positive, to go TOWARDS the other
            accelMag_separation = SimLaw.separation *   -1/distToNearest^2; %Negative, to go AWAY from the other
            accelMag_alignment  = SimLaw.alignment  *    1/distToNearest^2;
            accelMag_migration  = SimLaw.migration  *      distToTarget^6;
        
            newAccel = accelMag_separation * nearestUnit + ...
                       accelMag_cohesion   * centroidUnit + ...
                       accelMag_alignment  * nearestVelUnit + ...
                       accelMag_migration  * targetUnit; % nets accel vector to add on to current accel

            newAccel = newAccel.*[1,1,0]; % removes z component
    
            forwardUnit      = [cos(obj.heading);sin(obj.heading);0];
            newAccel_forward = dot(newAccel,forwardUnit);
            newAccel_circ    = 1.0 * sqrt((norm(newAccel))^2-newAccel_forward^2);
            
            turningCrossProduct = cross(forwardUnit,newAccel);
            newAccel_circ = newAccel_circ * sign(turningCrossProduct(3));

            newAccel = [newAccel_forward; newAccel_circ];

            %% Get Vel
            newVel(1) = obj.velocity(1) + newAccel(1)*SimLaw.dt;
            if(newVel(1) > SimLaw.forwardSpeedMax)
                newVel(1) = SimLaw.forwardSpeedMax;
            elseif(newVel(1) < SimLaw.forwardSpeedMin)
                newVel(1) = SimLaw.forwardSpeedMin;
            end
            % a = omega * v
            obj.bankAngle = atan(newAccel(2)/SimLaw.g);

            if(obj.bankAngle > SimLaw.bankMax)
                obj.bankAngle = SimLaw.bankMax;
            end
            if(obj.bankAngle < SimLaw.bankMin)
                obj.bankAngle = SimLaw.bankMin;
            end
            newAccel(2) = tan(obj.bankAngle*SimLaw.g);
            newVel(2) = newAccel(2)/newVel(1);

            vsink = (SimLaw.Sink_A*newVel(1).^2 + SimLaw.Sink_B*newVel(1) + SimLaw.Sink_C)...
                    / sqrt(cos(obj.bankAngle));
            vspeed = -vsink + thermalStrength;

            %% Get Pos
            newPos(1:2) = obj.position(1:2) + newVel(1)*forwardUnit(1:2)'*SimLaw.dt;
            newPos(3) = obj.position(3) + vspeed*SimLaw.dt;
            obj.heading = obj.heading + newVel(2)*SimLaw.dt;

            obj.velocity = newVel;
            obj.position = newPos;

        end
        
        % function 2
        function render(obj)
            rotationMatrix = [cos(obj.heading), -sin(obj.heading); sin(obj.heading), cos(obj.heading)];
            shape = SimLaw.agentShape_plane .* SimLaw.renderScale;
            rotatedShape = rotationMatrix * shape; %[x;y] matrix
            rotatedShape = rotatedShape'; %Convert to [x,y];
            globalShape = rotatedShape + obj.position(1:2); %[x,y] matrix
            
            if(class(obj.patchObj) == "double")
                obj.patchObj = patch('FaceColor','k');
            end
            obj.patchObj.XData = globalShape(:,1);
            obj.patchObj.YData = globalShape(:,2);
        end
    end
end