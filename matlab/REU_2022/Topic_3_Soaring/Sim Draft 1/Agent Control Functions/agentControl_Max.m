function agentControl_Max(thisAgent, localAgents, thermalStrength, target, SL)
%% Define Items
numNeighbors = size(localAgents,2);
thisPos = thisAgent.savedPosition;
thisVelFTZ = thisAgent.savedVelocity;
thisVelXY  = thisVelFTZ(1).*[cos(thisAgent.heading), sin(thisAgent.heading)];
cohesionVEC = [0 0];
separationVEC = [0 0];
alignmentVEC = [0 0];
% cohesionVEC   = thisAgent.cVEC;
% separationVEC = thisAgent.sVEC;
% alignmentVEC  = thisAgent.aVEC;

%% Get SCA Vectors...
if (numNeighbors > 0)% && (convertCharsToStrings(class(localAgents))=="Agent")
    %% Define More Items
    theirPos     = NaN(numNeighbors,3);
    theirHeading = NaN(numNeighbors,1);
    theirVelFTZ  = NaN(numNeighbors,3); %Forward Turning Z
    separationVecs = NaN(numNeighbors,2);
    alignmentVecs = NaN(numNeighbors,2);
    

    %% Collect Data from Neighbors
    for i = 1:numNeighbors
        theirPos(i,:) = localAgents(i).savedPosition;
        theirHeading(i) = localAgents(i).savedHeading;
        theirVelFTZ(i,:) = localAgents(i).savedVelocity;
%         assert(localAgents(i).isAlive,"Neighborhood function bronk")
    end
    
    diff = theirPos(:,1:3) - thisPos(1:3); % vectorized, this to them
    diff2D = diff(:,1:2);

    dist = vecnorm(diff,2,2); % 2-norm (Euclidean), 2nd dimension
    thisAgent.clearance = min(dist);
    dist2D = vecnorm(diff2D,2,2);
%     ndiff = diff./dist;
    ndiff2D = diff2D./dist2D;
    
    theirVelXY = theirVelFTZ(:,1).*[cos(theirHeading), sin(theirHeading)];

    theirRelVelZ = theirVelFTZ(:,3) - thisVelFTZ(3);
    
    %% Collision Death

    if any(dist < SL.collisionKillDistance)
        thisAgent.markedForDeath = true;
        return;
    end
    
    %% Cohesion
    ascendCaringFactor = SL.cohesionAscensionMult*(1 - (thisAgent.savedPosition(3)/SL.agentCeiling));

    % Cap relative Z
    theirRelVelZ(theirRelVelZ > SL.cohesionAscensionMax) = SL.cohesionAscensionMax;
   
    %Linear interp from [SL.cohesionAscensionIgnore,SL.cohesionAscensionMax] 
    %                to [1                         ,ascendCaringFactor]
    weightCohesion = (theirRelVelZ - SL.cohesionAscensionIgnore)*(ascendCaringFactor-1)/(SL.cohesionAscensionMax - SL.cohesionAscensionIgnore) + 1;
    % Set weights of sinking/weakly thermalling agents to 1
    weightCohesion(theirRelVelZ <= SL.cohesionAscensionIgnore) = 1;
    cohesionVecs = weightCohesion.*ndiff2D.*dist2D.^1;
    cohesionVEC  = sum(cohesionVecs,1) / numNeighbors;
%     thisAgent.cVEC = cohesionVEC;
    
    %% Separation and Alignment
    seenSA = abs(diff(:,3)) < SL.separationHeightWidth/2;
    separationVecs(seenSA,:) = - ndiff2D(seenSA,:)./dist2D(seenSA,:).^2;
    separationVecs(~seenSA,:) = [];
    separationVEC = sum(separationVecs,1);
%     thisAgent.sVEC = separationVEC;

    alignmentVecs(seenSA,:) = (theirVelXY(seenSA,:)-thisVelXY)./dist2D(seenSA,:).^2;
    alignmentVecs(~seenSA,:) = [];
    alignmentVEC = sum(alignmentVecs,1);
%     thisAgent.aVEC = alignmentVEC;
end

%% Migration
diffTarget = target - thisPos;
diffTarget(3) = [];
distTarget = norm(diffTarget);
migrationVEC = diffTarget*distTarget^5;

%% Waggle
if(thisAgent.lastWaggle <= 0)
    thisAgent.waggleSign = 2 * round(rand()) - 1;
    thisAgent.lastWaggle = Utility.randIR(SL.waggleDurationRange(1),SL.waggleDurationRange(2)); %s
end
thisAgent.lastWaggle = thisAgent.lastWaggle - SL.dt;
sideUnit = [cos(thisAgent.savedHeading + pi/2), sin(thisAgent.savedHeading + pi/2)];
waggleVEC = sideUnit * thisAgent.waggleSign;    


%% Get Accel
newAccel = separationVEC * SL.separation + ...
           cohesionVEC   * SL.cohesion   + ...
           alignmentVEC  * SL.alignment  + ...
           migrationVEC  * SL.migration  + ... 
           waggleVEC     * SL.waggle;

thisAgent.accelDir = atan2(newAccel(2), newAccel(1));

thisAgent.rulesDir(1) = atan2(separationVEC(2), separationVEC(1));
thisAgent.rulesDir(2) = atan2(cohesionVEC(2)  , cohesionVEC(1)  );
thisAgent.rulesDir(3) = atan2(alignmentVEC(2) , alignmentVEC(1) );
thisAgent.rulesDir(4) = atan2(migrationVEC(2) , migrationVEC(1) );
thisAgent.rulesDir(5) = atan2(waggleVEC(2)    , waggleVEC(1)    );

thisAgent.rulesMag(1) = norm(separationVEC * SL.separation);
thisAgent.rulesMag(2) = norm(cohesionVEC   * SL.cohesion  );
thisAgent.rulesMag(3) = norm(alignmentVEC  * SL.alignment );
thisAgent.rulesMag(4) = norm(migrationVEC  * SL.migration );
thisAgent.rulesMag(5) = norm(waggleVEC     * SL.waggle    );

forwardUnit = [cos(thisAgent.heading), sin(thisAgent.heading)];
newAccel_forward = dot(newAccel,forwardUnit);
if(norm(newAccel) - norm(newAccel_forward) < 1E-6)
    newAccel_circ = 0;
else
    newAccel_circ = 1.0 * sqrt((norm(newAccel))^2-newAccel_forward^2);
end
turningCrossProduct = cross([forwardUnit, 0],[newAccel, 0]);
newAccel_circ = newAccel_circ * sign(turningCrossProduct(3));

newAccel = [newAccel_forward; newAccel_circ];

%% Get Vel
newVel(1) = thisAgent.velocity(1) + newAccel(1)*SL.dt/10;
if(newVel(1) > SL.forwardSpeedMax)
    newVel(1) = SL.forwardSpeedMax;
elseif(newVel(1) < SL.forwardSpeedMin)
    newVel(1) = SL.forwardSpeedMin;
end
% a = omega * v
thisAgent.bankAngle = atan(newAccel(2)/SL.g);
thisAgent.bankAngle = thisAgent.bankAngle + SL.thermalBankFactor*thermalStrength*sign(thisAgent.bankAngle);
if(thisAgent.bankAngle > SL.bankMax)
    thisAgent.bankAngle = SL.bankMax;
end
if(thisAgent.bankAngle < SL.bankMin)
    thisAgent.bankAngle = SL.bankMin;
end
newAccel(2) = tan(thisAgent.bankAngle)*SL.g;
newVel(2) = newAccel(2)/newVel(1);

thisAgent.vsink = (SL.Sink_A*newVel(1).^2 + SL.Sink_B*newVel(1) + SL.Sink_C)...
        / sqrt(cos(thisAgent.bankAngle));
vspeed = thisAgent.vsink + thermalStrength;

%% Get Pos
newPos(1:2) = thisAgent.position(1:2) + newVel(1)*forwardUnit(1:2)*SL.dt;
newPos(3) = thisAgent.position(3) + vspeed*SL.dt;
if newPos(3) > SL.agentCeiling
    newPos(3) = SL.agentCeiling;
elseif newPos(3) < SL.agentFloor % not Giga-Jank
    if newPos(3) <= 0
        thisAgent.markedForDeath = true;
        %thisAgent.causeOfDeath = 'Ground';
        return;
    end
    newPos (3) = SL.agentFloor; % Tera-Jank
end

thisAgent.heading = thisAgent.heading + newVel(2)*SL.dt;

thisAgent.velocity(1:2) = newVel;
thisAgent.velocity(3)   = vspeed;
thisAgent.position = newPos;
assert(~isnan(newPos(3)),'ur a nan')
end