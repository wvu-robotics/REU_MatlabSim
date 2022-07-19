function agentControl_Adam(currentAgent, localAgents, thermalStrength, target, SL)
    
    %% Define vectors
    vector_cohesion = [0,0,0];
    vector_separation = [0,0,0];
    vector_alignment = [0,0,0];
    vector_migration = [0,0,0];
    vector_waggle = [0,0,0];
    
    numLocalAgents = size(localAgents,2);
    
    %{
    %% Non-neighbor Factors
    % Height desire
    height = currentAgent.savedPosition(3);
    height = min(max(height,SL.heightDesireBounds(1)),SL.heightDesireBounds(2));
    factor_heightDesire = interp1(SL.heightDesireBounds,SL.heightDesireMagBounds,height);
    %}
    
    
    % Height
    height = currentAgent.savedPosition(3);
    if(height > SL.heightDesireBounds(2))
        height = SL.heightDesireBounds(2);
    elseif(height < SL.heightDesireBounds(1))
        height = SL.heightDesireBounds(1);
    end
    factor_height = (height-SL.heightDesireBounds(1))/(SL.heightDesireBounds(2)-SL.heightDesireBounds(1));
        
    %% Neighbor Rules
    if(numLocalAgents > 0)
        otherPos = zeros(numLocalAgents,3);
        otherHeading = zeros(numLocalAgents,1);
        otherVel = zeros(numLocalAgents,3);
        
        for i=1:numLocalAgents
            otherPos(i,:) = localAgents(i).savedPosition;
            otherHeading(i) = localAgents(i).savedHeading;
            otherVel(i,:) = localAgents(i).savedVelocity;
        end
        
        currentPos = currentAgent.savedPosition;
        currentHeading = currentAgent.savedHeading;
        currentVel = currentAgent.savedVelocity;
        
        diffPos = otherPos - currentPos;
        dist = vecnorm(diffPos,2,2);
        
        %% Collision Death
        if any(dist < SL.collisionKillDistance)
            currentAgent.markedForDeath = true;
            return;
        end
        
        diffPos2D = diffPos(:,1:2);
        dist2D = vecnorm(diffPos2D,2,2);
        diffPos2DUnit = [diffPos2D./dist2D,zeros(numLocalAgents,1)];
        diffVelXY = [otherVel(:,1).*cos(otherHeading(:))-currentVel(1)*cos(currentHeading),...
                     otherVel(:,1).*sin(otherHeading(:))-currentVel(1)*sin(currentHeading),...
                     zeros(numLocalAgents,1)];
        diffVelZ = otherVel(:,3) - currentVel(3);
        
        %% Factors
        % Relative Ascension
        relativeAscension = diffVelZ;
        relativeAscension(relativeAscension < SL.relativeAscensionBounds(1)) = SL.relativeAscensionBounds(1);
        relativeAscension(relativeAscension > SL.relativeAscensionBounds(2)) = SL.relativeAscensionBounds(2);
        factor_relativeAscension = (relativeAscension-SL.relativeAscensionBounds(1))/(SL.relativeAscensionBounds(2)-SL.relativeAscensionBounds(1));
        
        % Relative Height
        relativeHeight = diffPos(:,3);
        % Already bounded by neighborRadius
        factor_relativeHeight = (relativeHeight-SL.neighborRadius)/(2*SL.neighborRadius);
        
        % Distance
        factor_distance = dist2D / SL.neighborRadius;
        
        %% Rules
        % Cohesion
        weight_cohesion = (factor_relativeAscension * (SL.coh_relativeAscension(2)-SL.coh_relativeAscension(1)) + SL.coh_relativeAscension(1)) .* ...
                          (factor_height * (SL.coh_heightDesire(2)-SL.coh_heightDesire(1)) + SL.coh_heightDesire(1)) .* ...
                          factor_distance;
        vectorAll_cohesion = (weight_cohesion .* diffPos2DUnit) / numLocalAgents;
        vector_cohesion = sum(vectorAll_cohesion,1);
        
        % Separation
        weight_separation = (factor_relativeHeight * (SL.sep_relativeHeight(2)-SL.sep_relativeHeight(1)) + SL.sep_relativeHeight(1)) .* ...
                            (factor_height * (SL.sep_heightDesire(2)-SL.sep_heightDesire(1)) + SL.sep_heightDesire(1)) .* ...
                            (1-factor_distance).^SL.avoidPower;
        vectorAll_separation = (weight_separation .* -diffPos2DUnit) / numLocalAgents;
        vector_separation = sum(vectorAll_separation,1);
        
        % Alignment
        weight_alignment = (factor_relativeHeight * (SL.align_relativeHeight(2)-SL.align_relativeHeight(1)) + SL.align_relativeHeight(1)) .* ...
                            (factor_height * (SL.align_heightDesire(2)-SL.align_heightDesire(1)) + SL.align_heightDesire(1)) .* ...
                            (1-factor_distance).^SL.avoidPower;
        vectorAll_alignment = (weight_alignment .* diffVelXY) / numLocalAgents;
        vector_alignment = sum(vectorAll_alignment,1);
        
        %{
        % Cohesion
        cohesionSum = [0,0,0];
        cohesionDiv = 0;
        
        % Separation
        separationSum = [0,0,0];
        separationDiv = 0;
        
        % Alignment
        alignmentSum = [0,0,0];
        alignmentDiv = 0;
        
        % Iterate through neighbors
        for i=1:numLocalAgents
            %% Set up useful variables
            localAgent = localAgents(i);
            
            otherPos = localAgent.savedPosition;
            otherPos2D = otherPos;
            otherPos2D(3) = 0;
            
            otherHeading = localAgent.savedHeading;
            otherVel = localAgent.savedVelocity(1) * [cos(otherHeading),sin(otherHeading),0];
            currentVel = currentAgent.savedVelocity(1) * [cos(currentAgent.savedHeading),sin(currentAgent.savedHeading),0];
            velDiff = otherVel - currentVel;
            
            diff = localAgent.savedPosition - currentAgent.savedPosition;
            diffHeight = diff(3);
            diff2D = diff;
            diff2D(3) = 0;
            diffDist = norm(diff);
            diff2DDist = norm(diff2D);
            diff2DUnit = diff2D/diff2DDist;
            
            %% Collision Death
            if(diffDist < SL.collisionKillDistance)
                currentAgent.markedForDeath = true;
                fprintf("Agent collided and died. :(\n");
                return;
            end
            
            %% Set up factors
            % Relative ascension
            relativeAscension = localAgent.savedVelocity(3) - currentAgent.savedVelocity(3);
            relativeAscension = min(max(relativeAscension,SL.relativeAscensionBounds(1)),SL.relativeAscensionBounds(2));
            factor_relativeAscension = interp1(SL.relativeAscensionBounds,SL.relativeAscensionMagBounds,relativeAscension);
            
            % Relative height
            relativeHeight = diffHeight;
            relativeHeight = min(max(relativeHeight,-SL.neighborRadius),SL.neighborRadius);
            factor_relativeHeight = interp1([-SL.neighborRadius,SL.neighborRadius],SL.relativeHeightMagBounds,relativeHeight);
            
            % Distance
            factor_distance = diff2DDist / SL.neighborRadius;
            
            %% Cohesion
            weight_cohesion = interp1(SL.relativeAscensionMagBounds,SL.coh_relativeAscension,factor_relativeAscension) * ...
                              interp1(SL.heightDesireMagBounds,SL.coh_heightDesire,factor_heightDesire) * ...
                              factor_distance;
            cohesionSum = cohesionSum + weight_cohesion*diff2DUnit;
            cohesionDiv = cohesionDiv + 1;
            
            %% Separation
            weight_separation = interp1(SL.relativeHeightMagBounds,SL.sep_relativeHeight,factor_relativeHeight) * ...
                                interp1(SL.heightDesireMagBounds,SL.sep_heightDesire,factor_heightDesire) * ...
                                (1 - factor_distance)^SL.avoidPower;
            separationSum = separationSum - weight_separation*diff2DUnit;
            separationDiv = separationDiv + 1;
            
            %% Alignment  
            weight_alignment = interp1(SL.relativeHeightMagBounds,SL.align_relativeHeight,factor_relativeHeight) * ...
                                interp1(SL.heightDesireMagBounds,SL.align_heightDesire,factor_heightDesire) * ...
                                (1 - factor_distance)^SL.avoidPower;
            alignmentSum = alignmentSum + weight_alignment*velDiff*SL.dt;
            alignmentDiv = alignmentDiv + 1;
        end
        
        %% Cohesion
        if(cohesionDiv ~= 0)
            cohesionSum = cohesionSum / cohesionDiv;
            vector_cohesion = cohesionSum;
        end
        
        %% Separation
        if(separationDiv ~= 0)
            separationSum = separationSum/separationDiv;
            vector_separation = separationSum;
        end
        
        %% Alignment
        if(alignmentDiv ~= 0)
            alignmentSum = alignmentSum/alignmentDiv;
            vector_alignment = alignmentSum;
        end
        %}
    end
   
    %% Migration
    diffTarget = target - currentAgent.savedPosition;
    diffTarget(3) = 0;
    distTarget = norm(diffTarget);
    diffTargetUnit = diffTarget/distTarget;
    
    %{
    weight_migration = interp1(SL.heightDesireMagBounds,SL.mig_heightDesire,factor_heightDesire) * ...
                       distTarget^2;
    %}
    weight_migration = (factor_height * (SL.mig_heightDesire(2)-SL.mig_heightDesire(1)) + SL.mig_heightDesire(1)) * ...
                        distTarget^2;
    vector_migration = weight_migration * diffTargetUnit;

    %% Waggle
    if(currentAgent.lastWaggle <= 0)
        currentAgent.waggleSign = 2 * round(rand()) - 1;
        currentAgent.lastWaggle = Utility.randIR(SL.waggleDurationRange(1),SL.waggleDurationRange(2)); %s
    end
    currentAgent.lastWaggle = currentAgent.lastWaggle - SL.dt;
    sideUnit = [cos(currentAgent.savedHeading + pi/2), sin(currentAgent.savedHeading + pi/2), 0];
    vector_waggle = sideUnit * currentAgent.waggleSign;    
    
    %% Get Accel
    vector_separation = vector_separation * SL.separation;
    vector_alignment = vector_alignment * SL.alignment;
    vector_cohesion = vector_cohesion * SL.cohesion;
    vector_migration = vector_migration * SL.migration;
    vector_waggle = vector_waggle * SL.waggle;

    newAccel = vector_separation + ...
               vector_alignment + ...
               vector_cohesion + ...
               vector_migration + ...
               vector_waggle;

    newAccel(3) = 0; % removes z component
    currentAgent.accelDir = atan2(newAccel(2), newAccel(1));
    currentAgent.rulesDir(1) = atan2(vector_separation(2), vector_separation(1));
    currentAgent.rulesDir(2) = atan2(vector_cohesion(2), vector_cohesion(1));
    currentAgent.rulesDir(3) = atan2(vector_alignment(2), vector_alignment(1));
    currentAgent.rulesDir(4) = atan2(vector_migration(2), vector_migration(1));
    currentAgent.rulesDir(5) = atan2(vector_waggle(2), vector_waggle(1));
    currentAgent.rulesMag(1) = norm(vector_separation);
    currentAgent.rulesMag(2) = norm(vector_cohesion);
    currentAgent.rulesMag(3) = norm(vector_alignment);
    currentAgent.rulesMag(4) = norm(vector_migration);
    currentAgent.rulesMag(5) = norm(vector_waggle);

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
        vspeed = min(vspeed,0);
    elseif newPos(3) < SL.agentFloor % not Giga-Jank
        newPos (3) = SL.agentFloor; % Tera-Jank
        vspeed = max(vspeed,0);
    end
    if newPos(3) <= 0
        currentAgent.markedForDeath = true;
        return;
    end
    currentAgent.heading = currentAgent.heading + newVel(2)*SL.dt;

    currentAgent.velocity(1:2) = newVel;
    currentAgent.velocity(3) = vspeed;
    currentAgent.position = newPos;
    if(isnan(newPos(3)))
        fprintf("NAN\n");
    end
end