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
        %% Check other agents
            for other = 1:numAgents
                if (other == agent) || (telemetry(step,other,4) <= 0)
                    continue;
                end
                otherPos     = 0;
                otherTheta   = 0;
                otherVelUnit = 0;

                diffPos = otherPos - agentPos;
                distToOther = norm(diffPos);
                diffUnit = diffPos / distToOther; % unit vector
                
                % skip if right on top of each other (should change later)
                if (distToOther == 0 || distToOther > neighborRadius)
                    continue;
                end
                
                accelMag_separation = separation * -1/distToOther^2; %Negative, to go AWAY from the other
                accelMag_cohesion   = cohesion   *    distToOther^2; %Positive, to go TOWARDS the other
                accelMag_alignment  = alignment  *  1/distToOther^2;
                
                accel = accelMag_separation*diffUnit + accelMag_cohesion*diffUnit + accelMag_alignment*otherVelUnit;
                tempAccel = tempAccel + accel;

            end
            


            %% Enforce wall separation
            wallPoints = [[wallMag;agentPos(2)],[agentPos(1);wallMag],[-wallMag;agentPos(2)],[agentPos(1);-wallMag]];
            wallAccelUnits = [[-1;0],[0;-1],[1;1],[0;1]];
            wallAccel = [0;0];
            for i=1:4 % 4 walls
               wallPoint = wallPoints(:,i);
               diffPos = wallPoint - agentPos;
               distToWall= norm(diffPos);
               if(distToWall > neighborRadius)
                   continue;
               end
               accelMag_separationWall = separationWall * 1/distToWall^2;
               tempAccel = tempAccel + accelMag_separationWall * wallAccelUnits(:,i);
               wallAccel = wallAccel + accelMag_separationWall * wallAccelUnits(:,i);
            end
            

            %% Calculate desired heading

            %% Calculate desired speed
            
            %% Calculate vertical speed
            vsink = (a*obj.velocity(1).^2 + b*obj.velocity(1) + c)...
                    / sqrt(cos(obj.bankAngle*2*pi/180));
            vspeed = vsink + thermalStrength;
            %% Create New Telemetry
            forwardUnit = [cos(agentTheta);sin(agentTheta)];
            newAccel_forward = dot(tempAccel,forwardUnit);
            newAccel_theta = 1.0 * sqrt((norm(tempAccel))^2-newAccel_forward^2);
            
            tempAccel3D = [tempAccel(1),tempAccel(2),0];
            forwardUnit3D = [forwardUnit(1),forwardUnit(2),0];
            turningCrossProduct = cross(forwardUnit3D,tempAccel3D);
            newAccel_theta = newAccel_theta * sign(turningCrossProduct(3));
            
            newAccel = [newAccel_forward; newAccel_theta];
            
            if(norm(newAccel(1)) > maxForwardAccel)
               newAccel(1) = sign(newAccel(1)) * maxForwardAccel; 
            end
            
            if(norm(newAccel(2)) > maxAlpha)
                newAccel(2) = sign(newAccel(2)) * maxAlpha;
            end
            
            newVel(1) = agentVel(1) + newAccel(1)*dt;
            
            if(newVel(1) > maxForwardVel)
                newVel(1) = maxForwardVel;
            elseif(newVel(1) < minForwardVel)
                newVel(1) = minForwardVel;
            end
            
            newVel(2) = newAccel(2);
            
            if(norm(newVel(2)) > maxOmega)
                newVel(2) = sign(newVel(2)) * maxOmega;
            end
            
            newTele = agentPos + newVel(1)*forwardUnit*dt;
            newTele(3) = agentTheta + newVel(2)*dt;
            newTele(4) = agentAlti - sinkRate*dt;
            % giga jank
            
            if norm([newTele(1),newTele(2)]) < thermal(3)/2 && newTele(4) < thermal(5)
                newTele(4) = newTele(4) + thermal(4)*dt;
            end
            newTele(5) = newVel(1);
            newTele(6) = newVel(2);

        end
    end
end