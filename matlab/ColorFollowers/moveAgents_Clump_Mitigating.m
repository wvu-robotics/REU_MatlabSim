%% moveAgents_Clump_Mitigating(environment, unitToGoal, sensingRadius, neighborToGoalForceRatio, theta, mapSize, movePerStepFraction)
%   Description: This is the control function for the
%   Color_follower_Improved script. In this control algorithm, each robot
%   has a square scan radius and can see each other agent in that radius.
%   The robots are attracted to other robots of similar colors and repelled
%   by robots of different colors

function [environment, theta] = moveAgents_Clump_Mitigating(environment, unitToGoal, sensingRadius, neighborToGoalForceRatio, theta, mapSize, movePerStepFraction)

force = zeros([mapSize, mapSize, 2]);
force = force + unitToGoal;
alreadyMoved = zeros(mapSize);

for agent = 1:round(sum(sum(environment(:,:,4)))*movePerStepFraction)
    notFound = true;
    while notFound
        i = randi(mapSize); %x coordinate
        j = randi(mapSize); %y coordinate
        if environment(i, j, 4) == 1 && not(alreadyMoved(i,j))
            notFound = false;
        end
    end
    for k = -sensingRadius:sensingRadius %x offset in sensing range
        for l = -sensingRadius:sensingRadius %y offset in sensing range
            if i+k > 0 && i+k <= mapSize && j+l > 0 && j+l <= mapSize && k ~= 0 && l ~= 0 && environment(i+k, j+l, 4) == 1 %make sure not off the edge of the map and not on central agent
                %Find the angle to the neighbors goal position
                angularGoalDistToNeighbor = abs(theta(i+k, j+l) - theta(i, j));
                if angularGoalDistToNeighbor > pi
                    angularGoalDistToNeighbor = 2*pi - angularGoalDistToNeighbor;
                end
                %Find the unit vector pointing to that neighbor
                unitToNeighbor = [k, l]/norm([k, l]);
                %For each neighbor, add their force. Attraction if
                %the angular dist of their goal trajectories is between 0 and 90 and
                %repulsion if the anglular dist is between 90 and
                %180 deg
                %Only be attracted to agents of similar colors in
                %front and only be repelled by dissimilar colors
                %behind
                floatingMiddle = pi/2;
                if (abs(acos(dot(unitToNeighbor, [unitToGoal(i,j,1),unitToGoal(i,j,2)]))) <= pi/1.8 && angularGoalDistToNeighbor <= floatingMiddle) ...
                        || (abs(acos(dot(unitToNeighbor, [unitToGoal(i,j,1),unitToGoal(i,j, 2)]))) > pi/1.8 && angularGoalDistToNeighbor > floatingMiddle)
                    force(i, j, 1) = force(i, j, 1)+unitToNeighbor(1)*(floatingMiddle-angularGoalDistToNeighbor)/(pi-floatingMiddle)*neighborToGoalForceRatio;
                    force(i, j, 2) = force(i, j, 2)+unitToNeighbor(2)*(floatingMiddle-angularGoalDistToNeighbor)/(pi-floatingMiddle)*neighborToGoalForceRatio;
                end
            end
        end
    end
    %Now that the force has been added for each agent, we actually
    %need to move the agents around
    %We will try to move to the neighboring square that is closest
    %to the angle their force points, but if there is another agent
    %there, they will try the two adjacent squares
    forceAngle = mod(atan2(force(i,j,2),force(i,j,1)),2*pi);
    
    xBottomWall = false;
    yBottomWall = false;
    xTopWall = false;
    yTopWall = false;
    if i == 1
        xBottomWall = true;
    end
    if j == 1
        yBottomWall = true;
    end
    if i == mapSize
        xTopWall = true;
    end
    if j == mapSize
        yTopWall = true;
    end
    found = false;
    
    movePlan = [0, 0, 0, 0, 0];
    if forceAngle <= pi/8 || forceAngle > 15/8*pi
        movePlan = [1, 8, 2, 7, 3]; %Cell 1
    elseif forceAngle <=3/8*pi
        movePlan = [2, 1, 3, 8, 4]; %Cell 2
    elseif forceAngle <=5/8*pi
        movePlan = [3, 2, 4, 1, 5]; %Cell 3
    elseif forceAngle <=7/8*pi
        movePlan = [4, 3, 5, 2, 6]; %Cell 4
    elseif forceAngle <=9/8*pi
        movePlan = [5, 4, 6, 3, 7]; %Cell 5
    elseif forceAngle <=11/8*pi
        movePlan = [6, 5, 7, 4, 8]; %Cell 6
    elseif forceAngle <=13/8*pi
        movePlan = [7, 6, 8, 5, 1]; %Cell 7
    elseif forceAngle <=15/8*pi
        movePlan = [8, 7, 1, 6, 2]; %Cell 8
    end
    for choice = 1:length(movePlan)
        if movePlan(choice) == 1 && not(found) && not(xTopWall) && environment(i+1, j, 4) == 0
            environment(i+1, j, :) = environment(i, j, :);
            environment(i, j, :) = zeros(size(environment(i, j, :)));
            theta(i+1, j) = theta(i, j);
            found = true;
            alreadyMoved(i+1,j) = true;
        elseif movePlan(choice) == 2 && not(found) && not(xTopWall) && not(yTopWall) && environment(i+1, j+1, 4) == 0
            environment(i+1, j+1, :) = environment(i, j, :);
            environment(i, j, :) = zeros(size(environment(i, j, :)));
            theta(i+1, j+1) = theta(i, j);
            found = true;
            alreadyMoved(i+1,j+1) = true;
        elseif movePlan(choice) == 3 && not(found) && not(yTopWall) && environment(i, j+1, 4) == 0
            environment(i, j+1, :) = environment(i, j, :);
            environment(i, j, :) = zeros(size(environment(i, j, :)));
            theta(i, j+1) = theta(i, j);
            found = true;
            alreadyMoved(i,j+1) = true;
        elseif movePlan(choice) == 4 && not(found) && not(yTopWall) && not(xBottomWall) && environment(i-1, j+1, 4) == 0
            environment(i-1, j+1, :) = environment(i, j, :);
            environment(i, j, :) = zeros(size(environment(i, j, :)));
            theta(i-1, j+1) = theta(i, j);
            found = true;
            alreadyMoved(i-1,j+1) = true;
        elseif movePlan(choice) == 5 && not(found) && not(xBottomWall) && environment(i-1, j, 4) == 0
            environment(i-1, j, :) = environment(i, j, :);
            environment(i, j, :) = zeros(size(environment(i, j, :)));
            theta(i-1, j) = theta(i, j);
            found = true;
            alreadyMoved(i-1,j) = true;
        elseif movePlan(choice) == 6 && not(found) && not(xBottomWall) && not(yBottomWall) && environment(i-1, j-1, 4) == 0
            environment(i-1, j-1, :) = environment(i, j, :);
            environment(i, j, :) = zeros(size(environment(i, j, :)));
            theta(i-1, j-1) = theta(i, j);
            found = true;
            alreadyMoved(i-1,j-1) = true;
        elseif movePlan(choice) == 7 && not(found) && not(yBottomWall) && environment(i, j-1, 4) == 0
            environment(i, j-1, :) = environment(i, j, :);
            environment(i, j, :) = zeros(size(environment(i, j, :)));
            theta(i, j-1) = theta(i, j);
            found = true;
            alreadyMoved(i,j-1) = true;
        elseif movePlan(choice) == 8 && not(found) && not(xTopWall) && not(yBottomWall) && environment(i+1, j-1, 4) == 0
            environment(i+1, j-1, :) = environment(i, j, :);
            environment(i, j, :) = zeros(size(environment(i, j, :)));
            theta(i+1, j-1) = theta(i, j);
            found = true;
            alreadyMoved(i+1,j-1) = true;
        end
    end
end
end
