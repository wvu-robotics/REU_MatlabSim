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
        function obj = update(obj,localAgents,thermalStrength, target)
            % cant do anything here until I know what localAgents looks like
            % enjoy fake functions
            %% Get Centroid
            Centroid = obj.position;
            distance = zeros(1,size(localAgents));
            for i = 1:size(localAgents)
                if localAgents(i).position(3) <= 0
                    continue;
                end
                distance(i) = norm(obj.position - localAgents(i).position);
                Centroid = Centroid + localAgents(i).position*isnear(obj, localAgents(i), NaN);
            end
    
            distToCentroid = norm(obj.position - Centroid);
            centroidUnit   = (obj.position - Centroid) / distToCentroid;

            nearInd   = find(distance == min(distance));
            distToNearest  = isnear(obj, localAgents(nearInd), NaN);
            nearestUnit    = (obj.position - localAgents(nearInd).position) / distToNearest;
            
            %velUnit        = obj.velocity(1)*[cos(obj.heading), sin(obj.heading)];
            nearestVelUnit = localAgents(nearInd).velocity(1)*...
                             [cos(localAgents(nearInd).heading); sin(localAgents(nearInd).heading)];


            distToTarget   = norm(obj.position - target);
            targetUnit     = (obj.position - target) / distToTarget;
            
            %% Get Accel
            accelMag_separation = SimLaw.separation *   -1/distToNearest^2; %Negative, to go AWAY from the other
            accelMag_cohesion   = SimLaw.cohesion   *      distToCentroid^2; %Positive, to go TOWARDS the other
            accelMag_alignment  = SimLaw.alignment  *    1/distToNearest^2;
            accelMag_migration  = SimLaw.migration  * 1e-6*distToTarget^6;
        
            newAccel = accelMag_separation * nearestUnit + ...
                       accelMag_cohesion   * centroidUnit + ...
                       accelMag_alignment  * nearestVelUnit + ...
                       accelMag_migration  * targetUnit; % nets accel vector to add on to current accel
    
            forwardUnit      = [cos(obj.heading);sin(obj.heading)];
            newAccel_forward = dot(newAccel,forwardUnit);
            newAccel_circ    = 1.0 * sqrt((norm(newAccel))^2-newAccel_forward^2);
            newAccel3D       = [newAccel(1),newAccel(2),0];
            forwardUnit3D    = [forwardUnit(1),forwardUnit(2),0];
            
            turningCrossProduct = cross(forwardUnit3D,newAccel3D);
            newAccel_circ = newAccel_circ * sign(turningCrossProduct(3));

            newAccel = [newAccel_forward; newAccel_circ];

            if(norm(newAccel(1)) > maxForwardAccel)
               newAccel(1) = sign(newAccel(1)) * SimLaw.maxForwardAccel; 
            end

            %% Get Vel
            newVel(1) = obj.velocity(1) + newAccel(1)*dt;
            if(newVel(1) > SimLaw.maxForwardVel)
                newVel(1) = SimLaw.maxForwardVel;
            elseif(newVel(1) < SimLaw.minForwardVel)
                newVel(1) = SimLaw.minForwardVel;
            end
            % a = omega * v
            newVel(2) = newAccel(2)/newVel(1);
            if(norm(newVel(2)) > SimLaw.maxOmega)
                newVel(2) = sign(newVel(2)) * SimLaw.maxOmega;
                newAccel(2) = newVel(1)*newVel(2);
            end
            obj.bankAngle = atan(newAccel(2)/SimLaw.g);

            vsink = (simLaw.Sink_A*newVel(1).^2 + simLaw.Sink_B*newVel(1) + simLaw.Sink_C)...
                    / sqrt(cos(obj.bankAngle));
            vspeed = -vsink + thermalStrength;

            %% Get Pos
            newPos(1:2) = obj.position(1:2) + newVel(1)*forwardUnit*dt;
            newPos(3) = obj.position(3) + vspeed*dt;
            obj.heading = obj.heading + newVel(2)*dt;

            obj.velocity = newVel;
            obj.position = newPos;

        end
        
        function render(obj)
            rotationMatrix = [cos(obj.heading), -sin(obj.heading); sin(obj.heading), cos(obj.heading)];
            rotatedShape = rotationMatrix * SimLaw.agentShape_plane; %[x;y] matrix
            rotatedShape = rotatedShape'; %Convert to [x,y];
            globalShape = rotatedShape + obj.position(1:2); %[x,y] matrix
            
            if(isnan(obj.patchObj))
                obj.patchObj = patch('FaceColor','k');
            end
            obj.patchObj.XData = globalShape(:,1);
            obj.patchObj.YData = globalShape(:,2);
        end
    end
end