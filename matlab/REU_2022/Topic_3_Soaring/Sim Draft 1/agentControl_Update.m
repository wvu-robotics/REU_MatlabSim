function agentControl_Update(currentAgent,localAgents,thermalStrength, target)
    numLocalAgents = size(localAgents,2);
    if(numLocalAgents > 0)
        %% Get Centroid
        centroid = [0,0,0];
        distances = zeros(1,numLocalAgents);
        for i = 1:numLocalAgents
            if localAgents(i).savedPosition(3) <= 0
                continue;
            end
            distances(i) = norm(currentAgent.position - localAgents(i).savedPosition);
            centroid = centroid + localAgents(i).savedPosition;
        end
        centroid = centroid / numLocalAgents;

        %% Get Vectors
        % Cohesion
        diffCentroid = centroid - currentAgent.position;
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
            diffNearest    = localAgents(nearest).savedPosition - currentAgent.position;
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
        diffTarget     = target - currentAgent.position;
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
        currentAgent.accelDir = atan2(newAccel(2), newAccel(1));
    else
        newAccel = [0,0,0];
    end

    forwardUnit      = [cos(currentAgent.heading);sin(currentAgent.heading);0];
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
    newVel(1) = currentAgent.velocity(1) + newAccel(1)*SimLaw.dt;
    if(newVel(1) > SimLaw.forwardSpeedMax)
        newVel(1) = SimLaw.forwardSpeedMax;
    elseif(newVel(1) < SimLaw.forwardSpeedMin)
        newVel(1) = SimLaw.forwardSpeedMin;
    end
    % a = omega * v
    currentAgent.bankAngle = atan(newAccel(2)/SimLaw.g);

    if(currentAgent.bankAngle > SimLaw.bankMax)
        currentAgent.bankAngle = SimLaw.bankMax;
    end
    if(currentAgent.bankAngle < SimLaw.bankMin)
        currentAgent.bankAngle = SimLaw.bankMin;
    end
    newAccel(2) = tan(currentAgent.bankAngle)*SimLaw.g;
    newVel(2) = newAccel(2)/newVel(1);

    vsink = (SimLaw.Sink_A*newVel(1).^2 + SimLaw.Sink_B*newVel(1) + SimLaw.Sink_C)...
            / sqrt(cos(currentAgent.bankAngle));
    vspeed = vsink + thermalStrength;

    %% Get Pos
    newPos(1:2) = currentAgent.position(1:2) + newVel(1)*forwardUnit(1:2)'*SimLaw.dt;
    newPos(3) = currentAgent.position(3) + vspeed*SimLaw.dt;
    if newPos(3) > SimLaw.agentCeiling
        newPos(3) = SimLaw.agentCeiling;
    elseif newPos(3) < SimLaw.agentFloor % not Giga-Jank
        newPos(3) = SimLaw.agentFloor; % Tera-Jank
    end
    if newPos(3) <= 0
        currentAgent.isAlive = false;
    end
    currentAgent.heading = currentAgent.heading + newVel(2)*SimLaw.dt;

    currentAgent.velocity = newVel;
    currentAgent.position = newPos;
end