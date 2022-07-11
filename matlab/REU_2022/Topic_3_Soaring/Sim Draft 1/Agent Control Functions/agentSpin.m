function agentSpin(thisAgent, localAgents, thermalStrength, target, SL)

%% Get Vel
forwardUnit = [cos(thisAgent.heading), sin(thisAgent.heading)];
newVel(1) = (SL.forwardSpeedMax - SL.forwardSpeedMin)*thisAgent.heading/2/pi+(SL.forwardSpeedMin);
if(newVel(1) > SL.forwardSpeedMax)
    newVel(1) = SL.forwardSpeedMax;
elseif(newVel(1) < SL.forwardSpeedMin)
    newVel(1) = SL.forwardSpeedMin;
end
% a = omega * v
thisAgent.bankAngle = SL.bankMax;

thisAgent.vsink = 0;
vspeed = thisAgent.vsink;

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
newAccel(2) = tan(thisAgent.bankAngle)*SL.g;
newVel(2) = newAccel(2)/newVel(1);
thisAgent.heading = thisAgent.heading + newVel(2)*SL.dt;


thisAgent.velocity(1:2) = newVel;
thisAgent.velocity(3)   = vspeed;
thisAgent.position = newPos;
assert(~isnan(newPos(3)),'ur a nan')
end