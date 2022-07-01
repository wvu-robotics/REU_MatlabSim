function agentControl_Adam(currentAgent, localAgents, thermalStrength, target, SL)
    
    %% Define vectors
    vector_cohesion = [0,0,0];
    vector_separation = [0,0,0];
    vector_alignment = [0,0,0];
    vector_migration = [0,0,0];
    vector_waggle = [0,0,0];
    
    numLocalAgents = size(localAgents,2);
    
    %% Neighbor Rules
    if(numLocalAgents > 0)
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
            
            diff = localAgent.savedPosition - currentAgent.savedPosition;
            diffHeight = diff(3);
            diff2D = diff;
            diff2D(3) = 0;
            diff2DDist = norm(diff2D);
            
            relativeAscension = localAgent.savedVelocity(3) - currentAgent.savedVelocity(3);
            
            %% Separation            
            if(abs(diffHeight) < SL.separationHeightWidth/2)
                separationSum = separationSum - diff2D/diff2DDist * 1/diff2DDist^2;
                separationDiv = separationDiv + 1;
            end
            
            %% Alignment            
            if(abs(diffHeight) < SL.alignmentHeightWidth/2)
                alignmentSum = alignmentSum + otherVel * 1/diff2DDist^2;
                alignmentDiv = separationDiv + 1;
            end
            
            %% Cohesion
            if(relativeAscension > SL.cohesionAscensionIgnore)
                if(relativeAscension > SL.cohesionAscensionMax)
                    relativeAscension = SL.cohesionAscensionMax;
                end
                %Linear interp of relativeAscension from [SL.cohesionAscensionIgnore,SL.cohesionAscensionMax] to [1,SL.cohesionAscensionMult]
                weightCohesion = (relativeAscension - SL.cohesionAscensionIgnore)/(SL.cohesionAscensionMax - SL.cohesionAscensionIgnore)*(SL.cohesionAscensionMult-1) + 1;
            else
                weightCohesion = 0;
            end
            cohesionSum = cohesionSum + weightCohesion*otherPos2D;
            cohesionDiv = cohesionDiv + 1;
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
        
        %% Cohesion
        if(cohesionDiv ~= 0)
            cohesionSum = cohesionSum / cohesionDiv;
            vector_cohesion = cohesionSum - currentAgent.savedPosition;
            %vector_cohesion = vector_cohesion * norm(vector_cohesion);
        end
    end
   
    %% Migration
    diffTarget = target - currentAgent.savedPosition;
    diffTarget(3) = 0;
    vector_migration = diffTarget;

    %% Waggle
    if(currentAgent.lastWaggle <= 0)
        currentAgent.waggleSign = 2 * round(rand()) - 1;
        currentAgent.lastWaggle = Utility.randIR(SL.waggleDurationRange(1),SL.waggleDurationRange(2)); %s
    end
    currentAgent.lastWaggle = currentAgent.lastWaggle - SL.dt;
    sideUnit = [cos(currentAgent.savedHeading + pi/2), sin(currentAgent.savedHeading + pi/2), 0];
    vector_waggle = sideUnit * currentAgent.waggleSign;    
    
    %% Get Accel
    newAccel = vector_separation * SL.separation + ...
               vector_alignment  * SL.alignment + ...
               vector_cohesion   * SL.cohesion  + ...
               vector_migration  * SL.migration + ...
               vector_waggle     * SL.waggle;

    newAccel(3) = 0; % removes z component
    currentAgent.accelDir = atan2(newAccel(2), newAccel(1));
    currentAgent.rulesDir(1) = atan2(vector_separation(2), vector_separation(1));
    currentAgent.rulesDir(2) = atan2(vector_cohesion(2), vector_cohesion(1));
    currentAgent.rulesDir(3) = atan2(vector_alignment(2), vector_alignment(1));
    currentAgent.rulesDir(4) = atan2(vector_migration(2), vector_migration(1));
    currentAgent.rulesMag(1) = norm(vector_separation);
    currentAgent.rulesMag(2) = norm(vector_cohesion);
    currentAgent.rulesMag(3) = norm(vector_alignment);
    currentAgent.rulesMag(4) = norm(vector_migration);

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

    currentAgent.velocity(1:2) = newVel;
    currentAgent.velocity(3) = vspeed;
    currentAgent.position = newPos;
    if(isnan(newPos(3)))
        fprintf("NAN\n");
    end
end