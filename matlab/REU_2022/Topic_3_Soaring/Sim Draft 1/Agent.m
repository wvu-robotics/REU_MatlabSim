% Agent class
classdef Agent < handle
    properties
        position = [0.0, 0.0, 0.0]      %m, [x,y,z]
        heading = 0.0                   %rad
        bankAngle = 0.0                 %rad
        velocity = [0.0, 0.0]           %m/s, rad/s, [forward,omega]
        patchObj = NaN
        patchArr = NaN

        accelDir = 0.0;
        
        savedPosition = [0.0, 0.0, 0.0]
        savedVelocity = [0.0, 0.0]
        savedHeading = 0.0
    end
    
    methods
        % Function 1 : Updates telemetry of obj
        function obj = update(obj,localAgents,thermalStrength, target)
            numLocalAgents = size(localAgents,2);
            if(numLocalAgents > 0)
                %% Get Centroid
                centroid = [0,0,0];
                distances = zeros(1,numLocalAgents);
                for i = 1:numLocalAgents
                    if localAgents(i).savedPosition(3) <= 0
                        continue;
                    end
                    distances(i) = norm(obj.position - localAgents(i).savedPosition);
                    centroid = centroid + localAgents(i).savedPosition;
                end
                centroid = centroid / numLocalAgents;

                %% Get Vectors
                % Cohesion
                diffCentroid = centroid - obj.position;
                diffCentroid(3) = 0;
                distToCentroid = norm(diffCentroid);
                if distToCentroid == 0
                    centroidUnit = [0,0,0];
                else
                    centroidUnit   = diffCentroid / distToCentroid;
                end

                % Separation & Alignment
                nearest = find(distances == min(distances));
                if ~isempty(nearest)
                    nearest        = nearest(1);
                    diffNearest    = localAgents(nearest).savedPosition - obj.position;
                    diffNearest(3) = 0;
                    distToNearest  = norm(diffNearest);
                    nearestUnit    = diffNearest / distToNearest;
                    nearestVelUnit = [cos(localAgents(nearest).savedHeading), sin(localAgents(nearest).savedHeading), 0];
                else
                    distToNearest  = 1;
                    nearestUnit    = [0,0,0];
                    nearestVelUnit = [0,0,0];
                end

                % Migration
                diffTarget     = target - obj.position;
                diffTarget(3)  = 0;
                distToTarget   = norm(diffTarget);
                targetUnit     = diffTarget / distToTarget;

                %% Get Accel
                accelMag_cohesion   = SimLaw.cohesion   *      distToCentroid^2; %Positive, to go TOWARDS the other
                accelMag_separation = SimLaw.separation *   -1/distToNearest^2; %Negative, to go AWAY from the other
                accelMag_alignment  = SimLaw.alignment  *    1/distToNearest^2;
                accelMag_migration  = SimLaw.migration  *      distToTarget^6;

                newAccel = accelMag_separation * nearestUnit + ...
                           accelMag_cohesion   * centroidUnit + ...
                           accelMag_alignment  * nearestVelUnit + ...
                           accelMag_migration  * targetUnit; % nets accel vector to add on to current accel

                newAccel(3) = 0; % removes z component
                obj.accelDir = atan2(newAccel(2), newAccel(1));
            else
                newAccel = [0,0,0];
            end
    
            forwardUnit      = [cos(obj.heading);sin(obj.heading);0];
            newAccel_forward = dot(newAccel,forwardUnit);
            if(norm(newAccel) - norm(newAccel_forward) < 1E-6)
                newAccel_circ = 0;
            else
                newAccel_circ = 1.0 * sqrt((norm(newAccel))^2-newAccel_forward^2);
            end
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
            newAccel(2) = tan(obj.bankAngle)*SimLaw.g;
            newVel(2) = newAccel(2)/newVel(1);

            vsink = (SimLaw.Sink_A*newVel(1).^2 + SimLaw.Sink_B*newVel(1) + SimLaw.Sink_C)...
                    / sqrt(cos(obj.bankAngle));
            vspeed = vsink + thermalStrength;

            %% Get Pos
            newPos(1:2) = obj.position(1:2) + newVel(1)*forwardUnit(1:2)'*SimLaw.dt;
            newPos(3) = obj.position(3) + vspeed*SimLaw.dt;
            obj.heading = obj.heading + newVel(2)*SimLaw.dt;

            obj.velocity = newVel;
            obj.position = newPos;

        end
        
        % function 2
        function obj = render(obj)
            rotationMatrix = [cos(obj.heading), -sin(obj.heading); sin(obj.heading), cos(obj.heading)];
            AccelMatrix    = [cos(obj.accelDir), -sin(obj.accelDir); sin(obj.accelDir), cos(obj.accelDir)];
            shape = SimLaw.agentShape_plane .* SimLaw.renderScale;
            arrow = SimLaw.Arrow .* SimLaw.renderScale;
            rotatedShape = rotationMatrix * shape; %[x;y] matrix
            rotatedShape = rotatedShape'; %Convert to [x,y];
            rotatedArrow = AccelMatrix * arrow;
            rotatedArrow = rotatedArrow';
            globalShape = rotatedShape + obj.position(1:2); %[x,y] matrix
            globalArrow = rotatedArrow + obj.position(1:2);

            if(class(obj.patchObj) == "double")
                scaledAlti = ((obj.position(3)+25)/100);
                color = hsv2rgb([scaledAlti,1,1]);
                obj.patchObj = patch('FaceColor',color);
                obj.patchArr = patch('FaceColor',color);
            end
            obj.patchObj.XData = globalShape(:,1);
            obj.patchObj.YData = globalShape(:,2);
            obj.patchArr.XData = globalArrow(:,1);
            obj.patchArr.YData = globalArrow(:,2);
        end
        
        function obj = saveData(obj)
            obj.savedPosition = obj.position;
            obj.savedVelocity = obj.velocity;
            obj.savedHeading = obj.heading;
        end
    end
end