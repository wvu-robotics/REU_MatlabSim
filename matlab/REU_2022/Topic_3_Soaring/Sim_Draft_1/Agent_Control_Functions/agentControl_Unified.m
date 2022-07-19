function agentControl_Unified(currentAgent, localAgents, thermalStrength, target, SL)
%% Define Items
numLocalAgents = size(localAgents,2);
currentPosXYZ = currentAgent.savedPosition;
height = currentPosXYZ(3);
currentVelFTZ = currentAgent.savedVelocity;
currentVelXY  = currentVelFTZ(1).*[cos(currentAgent.heading), sin(currentAgent.heading)];
vector_cohesion   = [0 0];
vector_separation = [0 0];
vector_alignment  = [0 0];

%% Get SCA Vectors...
if (numLocalAgents > 0)
    %% Define More Items
    theirPosXYZ     = NaN(numLocalAgents,3);
    theirHeading = NaN(numLocalAgents,1);
    theirVelFTZ  = NaN(numLocalAgents,3); %Forward Turning Z
    separationVecs = NaN(numLocalAgents,2);
    alignmentVecs = NaN(numLocalAgents,2);
    
    %% Collect Data from Neighbors
    for i = 1:numLocalAgents
        theirPosXYZ(i,:) = localAgents(i).savedPosition;
        theirHeading(i) = localAgents(i).savedHeading;
        theirVelFTZ(i,:) = localAgents(i).savedVelocity;
    end
    
    relPosXYZ = theirPosXYZ(:,1:3) - currentPosXYZ(1:3); % vectorized, this to them
    relPosXY = relPosXYZ(:,1:2);

    distXYZ = vecnorm(relPosXYZ,2,2); % 2-norm (Euclidean), 2nd dimension
    ndistXYZ = distXYZ./SL.neighborRadius;
    distXY = vecnorm(relPosXY,2,2);
    ndiffXY = relPosXY./distXY;
    ndistXY = distXY./SL.neighborRadius;

    theirVelXY = theirVelFTZ(:,1).*[cos(theirHeading), sin(theirHeading)];
    theirRelVelXY = theirVelXY - currentVelXY;
    theirRelVelZ = theirVelFTZ(:,3) - currentVelFTZ(3);

    currentAgent.clearance = min(distXYZ);
    currentAgent.neighborData.relPosition = relPosXYZ;
    currentAgent.neighborData.relVelocity = [theirVelXY - currentVelXY, theirRelVelZ];

    %% Collision Death

    if any(distXYZ < SL.collisionKillDistance)
        currentAgent.markedForDeath = true;
        return;
    end
    
    %% Cohesion
    % What you have to work with:
    % - Their Relative Ascension
    % - Your Absolute Height
    % - Their Distance

    % Step 1: Restrict Relative Ascension to Bounds (if 11, then 10)
    theirRelVelZ(theirRelVelZ > SL.cohesionAscensionMax) = SL.cohesionAscensionMax;
    theirRelVelZ(theirRelVelZ < SL.cohesionAscensionIgnore) = SL.cohesionAscensionIgnore;
    
    % Step 2: Gather Weight Multipliers from Height and Ascension (both 0-1)
    heightFactor    = (1 - height/SL.agentCeiling)^SL.heightFactorPower;
    normAscension   = (theirRelVelZ - SL.cohesionAscensionIgnore) / (SL.cohesionAscensionMax - SL.cohesionAscensionIgnore);
    ascensionFactor = normAscension.^SL.ascensionFactorPower;
    
    % Step 3: Get Result Cohesion Weighting
    weightCohesion = heightFactor*ascensionFactor;
    currentAgent.neighborData.cWeight = weightCohesion;
    cohesionVecs = weightCohesion.*ndiffXY.*ndistXY.^SL.cohPower;
    vector_cohesion  = sum(cohesionVecs,1) / numLocalAgents;
    
    %% Separation and Alignment
    seenS = abs(relPosXYZ(:,3)) < SL.separationHeightWidth/2;
    seenA = abs(relPosXYZ(:,3)) < SL.alignmentHeightWidth/2;
    separationVecs(seenS,:) = - ndiffXY(seenS,:).*ndistXYZ(seenS,:).^SL.sepPower;
    separationVecs(~seenS,:) = [];
    vector_separation = sum(separationVecs,1) / numLocalAgents;

    alignmentVecs(seenA,:) = theirRelVelXY(seenA,:).*ndistXY(seenA,:).^SL.aliPower;
    alignmentVecs(~seenA,:) = [];
    vector_alignment = sum(alignmentVecs,1) / numLocalAgents;
end

%% Migration
diffTarget = target - currentPosXYZ;
diffTarget(3) = [];
distTarget = norm(diffTarget);
migrationVEC = diffTarget*distTarget^SL.migPower;

%% Waggle
if(currentAgent.lastWaggle <= 0)
    currentAgent.waggleSign = 2 * round(rand()) - 1;
    currentAgent.lastWaggle = Utility.randIR(SL.waggleDurationRange(1),SL.waggleDurationRange(2)); %s
end
currentAgent.lastWaggle = currentAgent.lastWaggle - SL.dt;
sideUnit = [cos(currentAgent.savedHeading + pi/2), sin(currentAgent.savedHeading + pi/2)];
waggleVEC = sideUnit * currentAgent.waggleSign;    

%% Get Accel
newAccel = vector_separation * SL.separation + ...
           vector_cohesion   * SL.cohesion   + ...
           vector_alignment  * SL.alignment  + ...
           migrationVEC  * SL.migration  + ... 
           waggleVEC     * SL.waggle;

currentAgent.accelDir = atan2(newAccel(2), newAccel(1));

currentAgent.rulesDir(1) = atan2(vector_separation(2), vector_separation(1));
currentAgent.rulesDir(2) = atan2(vector_cohesion(2)  , vector_cohesion(1)  );
currentAgent.rulesDir(3) = atan2(vector_alignment(2) , vector_alignment(1) );
currentAgent.rulesDir(4) = atan2(migrationVEC(2) , migrationVEC(1) );
currentAgent.rulesDir(5) = atan2(waggleVEC(2)    , waggleVEC(1)    );

currentAgent.rulesMag(1) = norm(vector_separation * SL.separation);
currentAgent.rulesMag(2) = norm(vector_cohesion   * SL.cohesion  );
currentAgent.rulesMag(3) = norm(vector_alignment  * SL.alignment );
currentAgent.rulesMag(4) = norm(migrationVEC  * SL.migration );
currentAgent.rulesMag(5) = norm(waggleVEC     * SL.waggle    );

forwardUnit = [cos(currentAgent.heading), sin(currentAgent.heading)];
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
newVel(1) = currentAgent.velocity(1) + newAccel(1)*SL.dt/10;
if(newVel(1) > SL.forwardSpeedMax)
    newVel(1) = SL.forwardSpeedMax;
elseif(newVel(1) < SL.forwardSpeedMin)
    newVel(1) = SL.forwardSpeedMin;
end
% a = omega * v
currentAgent.bankAngle = atan(newAccel(2)/SL.g);
currentAgent.bankAngle = currentAgent.bankAngle + SL.thermalBankFactor*thermalStrength*sign(currentAgent.bankAngle);
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
    if newPos(3) <= 0
        currentAgent.markedForDeath = true;
        %thisAgent.causeOfDeath = 'Ground';
        return;
    end
    newPos (3) = SL.agentFloor; % Tera-Jank
end

currentAgent.heading = currentAgent.heading + newVel(2)*SL.dt;

currentAgent.velocity(1:2) = newVel;
currentAgent.velocity(3)   = vspeed;
currentAgent.position = newPos;


assert(~isnan(newPos(3)),'ur a nan')
end