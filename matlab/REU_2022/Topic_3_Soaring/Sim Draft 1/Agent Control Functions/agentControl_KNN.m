function agentControl_KNN(currentAgent, localAgents, thermalStrength, target, simLaw)
%{
thermalStrength = 10
mapSize = [-200,200]
numAgents = 20
separation = 15.0
cohesion = 0.06
waggle = 50
forwardSpeedMin = 15
forwardSpeedMax = 30
renderScale = [5;5];
%}

    numLocalAgents = size(localAgents,2);
    if(numLocalAgents > 0)
        %% Get Centroid
        % Centroid: 2D, scaled linearly by distance
        centroid = [0,0,0];
        distances2D = 1E6 * ones(1,numLocalAgents);
        centroidWeight = 0;
        for i = 1:numLocalAgents
            if localAgents(i).savedPosition(3) <= 0
                continue;
            end
            diffLocalAgent = localAgents(i).savedPosition - currentAgent.position;
            diffLocalAgent(3) = 0;
            distLocalAgent = norm(diffLocalAgent);
            distances2D(i) = distLocalAgent;
            scaledDist = distLocalAgent/simLaw.neighborRadius;
            weight = scaledDist;
            centroid = centroid + weight*localAgents(i).savedPosition;
            centroidWeight = centroidWeight + weight;
        end
        centroid = centroid / centroidWeight;

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

        % Find k-nearest neighbors(KNN)
        k = 10;
        if(numLocalAgents < k)
            NNIndices = 1:numLocalAgents;
            numNN = numLocalAgents;
        else
            [~,distSortIndices] = sort(distances2D);
            NNIndices = distSortIndices(1:k);
            numNN = k;
        end
        
        % Separation and alignment
        separationVector = [0,0,0];
        alignmentVector = [0,0,0];
        
        for i=1:numNN
            NNIndex = NNIndices(i);
            NNAgent = localAgents(NNIndex);
            diffNNAgent = NNAgent.savedPosition - currentAgent.position;
            diffNNAgent(3) = 0;
            distNNAgent = norm(diffNNAgent);
            distNNAgentScaled = distNNAgent/simLaw.neighborRadius;
            diffUnitNNAgent = diffNNAgent/distNNAgent;
            NNAgentHeading = NNAgent.savedHeading;
            
            velUnitNNAgent = [cos(NNAgentHeading),sin(NNAgentHeading),0];
            
            weight = 1/distNNAgentScaled^2;
            separationVector = separationVector - weight*diffUnitNNAgent;
            alignmentVector = alignmentVector + weight*velUnitNNAgent;
        end
        
        separationVector = separationVector / numNN;
        alignmentVector = alignmentVector / numNN;

        % Migration
        diffTarget     = target - currentAgent.position;
        diffTarget(3)  = 0;
        distToTarget   = norm(diffTarget);
        targetUnit     = diffTarget / distToTarget;

        % Waggle
        forwardUnit = [cos(currentAgent.heading), sin(currentAgent.heading), 0];
        upUnit = [0,0,1];
        sideUnit = cross(upUnit,forwardUnit);
        waggleSign = 2 * round(rand()) - 1;
        
        %% Get Accel
        accelMag_cohesion   = simLaw.cohesion * distToCentroid^2;
        accelMag_separation = simLaw.separation;
        accelMag_alignment  = simLaw.alignment;
        accelMag_migration  = simLaw.migration * distToTarget^6;
        accelMag_waggle     = simLaw.waggle * waggleSign;

        newAccel = accelMag_separation * separationVector + ...
                   accelMag_cohesion   * centroidUnit + ...
                   accelMag_alignment  * alignmentVector + ...
                   accelMag_migration  * targetUnit + ... % nets accel vector to add on to current accel
                   accelMag_waggle     * sideUnit;

        newAccel(3) = 0; % removes z component
        currentAgent.accelDir = atan2(newAccel(2), newAccel(1));
    else
        newAccel = [0,0,0];
    end

    forwardUnit = [cos(currentAgent.heading), sin(currentAgent.heading), 0];
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
    newVel(1) = currentAgent.velocity(1) + newAccel(1)*simLaw.dt;
    if(newVel(1) > simLaw.forwardSpeedMax)
        newVel(1) = simLaw.forwardSpeedMax;
    elseif(newVel(1) < simLaw.forwardSpeedMin)
        newVel(1) = simLaw.forwardSpeedMin;
    end
    % a = omega * v
    currentAgent.bankAngle = atan(newAccel(2)/simLaw.g);

    if(currentAgent.bankAngle > simLaw.bankMax)
        currentAgent.bankAngle = simLaw.bankMax;
    end
    if(currentAgent.bankAngle < simLaw.bankMin)
        currentAgent.bankAngle = simLaw.bankMin;
    end
    newAccel(2) = tan(currentAgent.bankAngle)*simLaw.g;
    newVel(2) = newAccel(2)/newVel(1);

    vsink = (simLaw.Sink_A*newVel(1).^2 + simLaw.Sink_B*newVel(1) + simLaw.Sink_C)...
            / sqrt(cos(currentAgent.bankAngle));
    vspeed = vsink + thermalStrength;

    %% Get Pos
    newPos(1:2) = currentAgent.position(1:2) + newVel(1)*forwardUnit(1:2)*simLaw.dt;
    newPos(3) = currentAgent.position(3) + vspeed*simLaw.dt;
    if newPos(3) > simLaw.agentCeiling
        newPos(3) = simLaw.agentCeiling;
    elseif newPos(3) < simLaw.agentFloor % not Giga-Jank
        newPos (3) = simLaw.agentFloor; % Tera-Jank
    end
    if newPos(3) <= 0
        currentAgent.isAlive = false;
    end
    currentAgent.heading = currentAgent.heading + newVel(2)*simLaw.dt;

    currentAgent.velocity = newVel;
    currentAgent.position = newPos;
    if(isnan(newPos(3)))
        fprintf("NAN\n");
    end
end