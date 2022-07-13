function agentControl_Max(currentAgent, localAgents, thermalStrength, target, SL)
%% Define Items
numNeighbors = size(localAgents,2);
thisPos = currentAgent.savedPosition;
thisVelFTZ = currentAgent.savedVelocity;
thisVelXY  = thisVelFTZ(1).*[cos(currentAgent.heading), sin(currentAgent.heading)];
cohesionVEC = [0 0];
separationVEC = [0 0];
alignmentVEC = [0 0];

%% Get SCA Vectors...
if (numNeighbors > 0)
    %% Define More Items
    theirPosXYZ     = NaN(numNeighbors,3);
    theirHeading = NaN(numNeighbors,1);
    theirVelFTZ  = NaN(numNeighbors,3); %Forward Turning Z
    separationVecs = NaN(numNeighbors,2);
    alignmentVecs = NaN(numNeighbors,2);
    

    %% Collect Data from Neighbors
    for i = 1:numNeighbors
        theirPosXYZ(i,:) = localAgents(i).savedPosition;
        theirHeading(i) = localAgents(i).savedHeading;
        theirVelFTZ(i,:) = localAgents(i).savedVelocity;
%         assert(localAgents(i).isAlive,"Neighborhood function bronk")
    end
    
    relPosXYZ = theirPosXYZ(:,1:3) - thisPos(1:3); % vectorized, this to them
    relPosXY = relPosXYZ(:,1:2);

    distXYZ = vecnorm(relPosXYZ,2,2); % 2-norm (Euclidean), 2nd dimension
    distXY = vecnorm(relPosXY,2,2);
    ndiffXY = relPosXY./distXY;

    theirVelXY = theirVelFTZ(:,1).*[cos(theirHeading), sin(theirHeading)];
    theirRelVelZ = theirVelFTZ(:,3) - thisVelFTZ(3);

    currentAgent.clearance = min(distXYZ);
    currentAgent.neighborData.relPosition = relPosXYZ;
    currentAgent.neighborData.relVelocity = [theirVelXY - thisVelXY, theirRelVelZ];

    %% Collision Death

    if any(distXYZ < SL.collisionKillDistance)
        currentAgent.markedForDeath = true;
        return;
    end
    
    %% Cohesion
    ascendCaringFactor = SL.cohesionAscensionMult*(1 - (thisPos(3)/SL.agentCeiling)); % 0 to whatever the max is

    % Cap relative Z
    theirRelVelZ(theirRelVelZ > SL.cohesionAscensionMax) = SL.cohesionAscensionMax;
   
    %Linear interp from [SL.cohesionAscensionIgnore,SL.cohesionAscensionMax] 
    %                to [1                         ,ascendCaringFactor]
    weightCohesion = (theirRelVelZ - SL.cohesionAscensionIgnore)*(ascendCaringFactor-1)/(SL.cohesionAscensionMax - SL.cohesionAscensionIgnore) + 1;
    % Set weights of sinking/weakly thermalling agents to 1
    weightCohesion(theirRelVelZ <= SL.cohesionAscensionIgnore) = 1;
    currentAgent.neighborData.cWeight = weightCohesion;
    cohesionVecs = weightCohesion.*ndiffXY.*distXY.^1;
    cohesionVEC  = sum(cohesionVecs,1) / numNeighbors;
    
    %% Separation and Alignment
    seenSA = abs(relPosXYZ(:,3)) < SL.separationHeightWidth/2;
    separationVecs(seenSA,:) = - ndiffXY(seenSA,:)./distXY(seenSA,:).^2;
    separationVecs(~seenSA,:) = [];
    separationVEC = sum(separationVecs,1);

    alignmentVecs(seenSA,:) = (theirVelXY(seenSA,:)-thisVelXY)./distXY(seenSA,:).^2;
    alignmentVecs(~seenSA,:) = [];
    alignmentVEC = sum(alignmentVecs,1);
end

%% Migration
diffTarget = target - thisPos;
diffTarget(3) = [];
distTarget = norm(diffTarget);
migrationVEC = diffTarget*distTarget^5;

%% Waggle
if(currentAgent.lastWaggle <= 0)
    currentAgent.waggleSign = 2 * round(rand()) - 1;
    currentAgent.lastWaggle = Utility.randIR(SL.waggleDurationRange(1),SL.waggleDurationRange(2)); %s
end
currentAgent.lastWaggle = currentAgent.lastWaggle - SL.dt;
sideUnit = [cos(currentAgent.savedHeading + pi/2), sin(currentAgent.savedHeading + pi/2)];
waggleVEC = sideUnit * currentAgent.waggleSign;    


%% Get Accel
newAccel = separationVEC * SL.separation + ...
           cohesionVEC   * SL.cohesion   + ...
           alignmentVEC  * SL.alignment  + ...
           migrationVEC  * SL.migration  + ... 
           waggleVEC     * SL.waggle;

currentAgent.accelDir = atan2(newAccel(2), newAccel(1));

currentAgent.rulesDir(1) = atan2(separationVEC(2), separationVEC(1));
currentAgent.rulesDir(2) = atan2(cohesionVEC(2)  , cohesionVEC(1)  );
currentAgent.rulesDir(3) = atan2(alignmentVEC(2) , alignmentVEC(1) );
currentAgent.rulesDir(4) = atan2(migrationVEC(2) , migrationVEC(1) );
currentAgent.rulesDir(5) = atan2(waggleVEC(2)    , waggleVEC(1)    );

currentAgent.rulesMag(1) = norm(separationVEC * SL.separation);
currentAgent.rulesMag(2) = norm(cohesionVEC   * SL.cohesion  );
currentAgent.rulesMag(3) = norm(alignmentVEC  * SL.alignment );
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