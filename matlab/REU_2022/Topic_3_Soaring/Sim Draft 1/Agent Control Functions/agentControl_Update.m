function agentControl_Update(currentAgent,localAgents,thermalStrength, target, SL)
    numLocalAgents = size(localAgents,2);

    % some common migration stuff
    diffTarget     = target - currentAgent.position;
    diffTarget(3)  = 0;
    distToTarget   = norm(diffTarget);
    targetUnit     = diffTarget / distToTarget;

    %% Get SCAM Accel
    if(numLocalAgents > 0)
        %% Get Centroid + Cohesion
        [centroid, distances, diffHeight] = Utility.findCentroid(currentAgent, localAgents, SL);

        % Cohesion
        diffCentroid    = centroid - currentAgent.position;
        diffCentroid(3) = 0;
        distToCentroid  = norm(diffCentroid);
        if distToCentroid == 0
            centroidUnit = [0,0,0]; % skips cohesion
        else
            centroidUnit = diffCentroid / distToCentroid;
        end

        %% Get Nearest

        % Nearest must be vertically close
        vertRange = 0.5; % 50% above and below.
        nearest   = 1;
        for i = 2:numLocalAgents
            if abs(diffHeight(i)) <= vertRange && distances(i) <= distances(i-1)
                nearest = i;
            end
        end

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
        
        %% Get SCAM Accel (with neighbors)

        accelMag_separation = SL.separation *   -1/distToNearest^2; %Negative, to go AWAY from the other
        accelMag_cohesion   = SL.cohesion   *      distToCentroid^2; %Positive, to go TOWARDS the other
        accelMag_alignment  = SL.alignment  *    1/distToNearest^2;
        accelMag_migration  = SL.migration  *      distToTarget^6;

        newAccel = accelMag_separation * nearestUnit    + ...
                   accelMag_cohesion   * centroidUnit   + ...
                   accelMag_alignment  * nearestVelUnit + ...
                   accelMag_migration  * targetUnit; % nets accel vector to add on to current accel
    else
        %% Cope with Loneliness
        % Agents will be directed solely by migration if alone.
        accelMag_migration  = SL.migration  *      distToTarget^6;
        newAccel = accelMag_migration  * targetUnit;
    end

    %% Finalize Accel
    newAccel(3) = 0; % removes z component
    currentAgent.accelDir = atan2(newAccel(2), newAccel(1));

    Heading      = [cos(currentAgent.heading),sin(currentAgent.heading),0];
    forwardAccel = dot(newAccel,Heading);
    if(norm(newAccel) - norm(forwardAccel) < 1E-6)
        CentriAccel = 0;
    else
        CentriAccel = 1.0 * sqrt((norm(newAccel))^2-forwardAccel^2);
    end
    turningCrossProduct = cross(Heading,newAccel);
    CentriAccel = CentriAccel * sign(turningCrossProduct(3));

    newAccel = [forwardAccel; CentriAccel];

    %% Get Vel
    newVel(1) = currentAgent.velocity(1) + newAccel(1)*SL.dt;

    newVel(1) = Utility.midMinMax(newVel(1),SL.forwardSpeedMin, SL.forwardSpeedMax);

    % a = omega * v
    currentAgent.bankAngle = atan(newAccel(2)/SL.g);
    
    currentAgent.bankAngle = Utility.midMinMax(currentAgent.bankAngle,SL.bankMin,  SL.bankMax);

    newAccel(2) = tan(currentAgent.bankAngle)*SL.g;
    newVel(2) = newAccel(2)/newVel(1);

    vsink = (SL.Sink_A*newVel(1).^2 + SL.Sink_B*newVel(1) + SL.Sink_C)...
            / sqrt(cos(currentAgent.bankAngle));
    vspeed = vsink + thermalStrength;

    %% Get Pos
    newPos(1:2) = currentAgent.position(1:2) + newVel(1)*Heading(1:2)*SL.dt;
    newPos(3)   = currentAgent.position(3)   + vspeed*SL.dt;
    newPos(3)   = Utility.midMinMax(newPos(3), SL.agentFloor, SL.agentCeiling);

    if newPos(3) <= 0
        currentAgent.isAlive = false;
    end
    currentAgent.heading = currentAgent.heading + newVel(2)*SL.dt;

    currentAgent.velocity = newVel;
    currentAgent.position = newPos;
end