function agentControl_Max(currentAgent, localAgents, thermalStrength, target, SL)
    accelMag_separation = 0;
    accelMag_cohesion = 0;
    accelMag_alignment = 0;
    accelMag_migration = 0;
    accelMag_waggle = 0;
    
    separationVector = [0,0,0];
    centroidUnit = [0,0,0];
    alignmentVector = [0,0,0];
    targetUnit = [0,0,0];
    sideUnit = [0,0,0];
    
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
            diffHeight = diffLocalAgent(3);
            diffLocalAgent(3) = 0;
            distLocalAgent = norm(diffLocalAgent);
            distances2D(i) = distLocalAgent;
            scaledDist = distLocalAgent/SL.neighborRadius;
            heightOffset = -3;
            if(diffHeight > heightOffset)
                weight = scaledDist * (SL.cohesionHeightMult*(diffHeight-heightOffset)/SL.neighborRadius + 1);
            else
                weight = 0;
            end
            centroid = centroid + weight*localAgents(i).savedPosition;
            centroidWeight = centroidWeight + weight;
        end
        centroid = centroid / centroidWeight;

        %% Get Vectors
        % Cohesion
        if(centroidWeight ~= 0)
            diffCentroid = centroid - currentAgent.position;
            diffCentroid(3) = 0;
            distToCentroid = norm(diffCentroid);
            if distToCentroid == 0
                centroidUnit = [0,0,0];
            else
                centroidUnit = diffCentroid / distToCentroid;
            end
            
            accelMag_cohesion   = SL.cohesion * distToCentroid^2;
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
            diffNNHeight = diffNNAgent(3);
            diffNNAgent(3) = 0;
            distNNAgent = norm(diffNNAgent);
            distNNAgentScaled = distNNAgent/SL.neighborRadius;
            diffUnitNNAgent = diffNNAgent/distNNAgent;
            NNAgentHeading = NNAgent.savedHeading;
            
            velUnitNNAgent = [cos(NNAgentHeading),sin(NNAgentHeading),0];
            
            weightSep = 1/distNNAgentScaled^2 - 1;
            weightAlign = 1;
            if(norm(diffNNHeight) > SL.separationHeightGap)
                weightSep = 0;
                weightAlign = 0;
            end
            separationVector = separationVector - weightSep*diffUnitNNAgent;
            alignmentVector = alignmentVector + weightAlign*velUnitNNAgent;
        end
        
        separationVector = separationVector / numNN;
        alignmentVector = alignmentVector / numNN;
        
        accelMag_separation = SL.separation;
        accelMag_alignment  = SL.alignment;
    end
   
    % Migration
    diffTarget = target - currentAgent.position;
    diffTarget(3) = 0;
    distToTarget = norm(diffTarget);
    targetUnit = diffTarget / distToTarget;

    % Waggle
    forwardUnit = [cos(currentAgent.heading), sin(currentAgent.heading), 0];
    upUnit = [0,0,1];
    sideUnit = cross(upUnit,forwardUnit);
    if(currentAgent.lastWaggle <= 0)
        currentAgent.waggleSign = 2 * round(rand()) - 1;
        currentAgent.lastWaggle = Utility.randIR(SL.waggleTime,1.5*SL.waggleTime); %s
    end
    currentAgent.lastWaggle = currentAgent.lastWaggle - SL.dt;
    
    accelMag_migration  = SL.migration * distToTarget^6;
    accelMag_waggle     = SL.waggle * currentAgent.waggleSign;
    
    
    %% Get Accel
    newAccel = accelMag_separation * separationVector + ...
               accelMag_cohesion   * centroidUnit + ...
               accelMag_alignment  * alignmentVector + ...
               accelMag_migration  * targetUnit + ... % nets accel vector to add on to current accel
               accelMag_waggle     * sideUnit;

    newAccel(3) = 0; % removes z component
    currentAgent.accelDir = atan2(newAccel(2), newAccel(1));
    currentAgent.rulesDir(1) = atan2(separationVector(2), separationVector(1));
    currentAgent.rulesDir(2) = atan2(centroidUnit(2), centroidUnit(1));
    currentAgent.rulesDir(3) = atan2(alignmentVector(2), alignmentVector(1));
    currentAgent.rulesDir(4) = atan2(targetUnit(2), targetUnit(1));
    currentAgent.rulesDir(5) = atan2(sideUnit(2), sideUnit(1));
    currentAgent.rulesMag(1) = accelMag_separation;
    currentAgent.rulesMag(2) = accelMag_cohesion;
    currentAgent.rulesMag(3) = accelMag_alignment;
    currentAgent.rulesMag(4) = accelMag_migration;
    currentAgent.rulesMag(5) = accelMag_waggle;

    

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
    newVel(1) = currentAgent.velocity(1) + newAccel(1)*SL.dt;
    if(newVel(1) > SL.forwardSpeedMax)
        newVel(1) = SL.forwardSpeedMax;
    elseif(newVel(1) < SL.forwardSpeedMin)
        newVel(1) = SL.forwardSpeedMin;
    end
    % a = omega * v
    currentAgent.bankAngle = atan(newAccel(2)/SL.g);

    if(currentAgent.bankAngle > SL.bankMax)
        currentAgent.bankAngle = SL.bankMax;
    end
    if(currentAgent.bankAngle < SL.bankMin)
        currentAgent.bankAngle = SL.bankMin;
    end
    newAccel(2) = tan(currentAgent.bankAngle)*SL.g;
    newVel(2) = newAccel(2)/newVel(1);

    currentAgent.vsink = (SL.Sink_A*newVel(1).^2 + SL.Sink_B*newVel(1) + SL.Sink_C)...
            / sqrt(cos(currentAgent.bankAngle));
    vspeed = currentAgent.vsink + thermalStrength;

    %% Get Pos
    newPos(1:2) = currentAgent.position(1:2) + newVel(1)*forwardUnit(1:2)*SL.dt;
    newPos(3) = currentAgent.position(3) + vspeed*SL.dt;
    if newPos(3) > SL.agentCeiling
        newPos(3) = SL.agentCeiling;
    elseif newPos(3) < SL.agentFloor % not Giga-Jank
        newPos (3) = SL.agentFloor; % Tera-Jank
    end
    if newPos(3) <= 0
        currentAgent.isAlive = false;
    end
    currentAgent.heading = currentAgent.heading + newVel(2)*SL.dt;

    currentAgent.velocity = newVel;
    currentAgent.position = newPos;
    if(isnan(newPos(3)))
        fprintf("NAN\n");
    end
end