%% moveAgents_Local_Diversity(environment, forceAngle, theta, mapSize, movePerStepFraction)
%   This function is used by the Color_Followers_Local_Diversity script to
%   actually move the agents once the appropriate dirrection has been
%   determened. It is a much inproved version of the previous color
%   follower movement method.
function [environment, theta] = moveAgents_Local_Diversity(environment, forceAngle, theta, mapSize, movePerStepFraction)

alreadyMoved = zeros(mapSize);

for agent = 1:round(sum(sum(environment.occupancy(:,:)))*movePerStepFraction)
    notFound = true;
    while notFound
        i = randi(mapSize); %x coordinate
        j = randi(mapSize); %y coordinate
        if environment.occupancy(i, j) == 1 && not(alreadyMoved(i,j))
            notFound = false;
        end
    end

    
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
    if forceAngle(i,j) <= pi/8 || forceAngle(i,j) > 15/8*pi
        movePlan = [1, 8, 2, 7, 3]; %Cell 1
    elseif forceAngle(i,j) <=3/8*pi
        movePlan = [2, 1, 3, 8, 4]; %Cell 2
    elseif forceAngle(i,j) <=5/8*pi
        movePlan = [3, 2, 4, 1, 5]; %Cell 3
    elseif forceAngle(i,j) <=7/8*pi
        movePlan = [4, 3, 5, 2, 6]; %Cell 4
    elseif forceAngle(i,j) <=9/8*pi
        movePlan = [5, 4, 6, 3, 7]; %Cell 5
    elseif forceAngle(i,j) <=11/8*pi
        movePlan = [6, 5, 7, 4, 8]; %Cell 6
    elseif forceAngle(i,j) <=13/8*pi
        movePlan = [7, 6, 8, 5, 1]; %Cell 7
    else
        movePlan = [8, 7, 1, 6, 2]; %Cell 8
    end
    
    move = [1, 1, 0, -1, -1, -1, 0, 1; 0, 1, 1, 1, 0, -1, -1, -1]; %First row add to X, second row add to Y for each move plan
    
    for choice = 1:length(movePlan)
        if ((move(1, movePlan(choice)) == 1 && not(xTopWall)) || (move(1, movePlan(choice)) == -1 && not(xBottomWall)) || (move(1, movePlan(choice)) == 0))...
                && ((move(2, movePlan(choice)) == 1 && not(yTopWall)) || (move(2, movePlan(choice)) == -1 && not(yBottomWall)) || (move(2, movePlan(choice)) == 0))...
                && not(found) && environment.occupancy(i+move(1, movePlan(choice)), j+move(2, movePlan(choice))) == 0
            environment.goalLocation(i+move(1, movePlan(choice)), j+move(2, movePlan(choice)), :) = environment.goalLocation(i, j, :);
            environment.goalLocation(i, j, :) = zeros(size(environment.goalLocation(i, j, :)));
            environment.occupancy(i+move(1, movePlan(choice)), j+move(2, movePlan(choice)), :) = environment.occupancy(i, j, :);
            environment.occupancy(i, j, :) = zeros(size(environment.occupancy(i, j, :)));
            theta(i+move(1, movePlan(choice)), j+move(2, movePlan(choice))) = theta(i, j);
            found = true;
            alreadyMoved(i+move(1, movePlan(choice)),j+move(2, movePlan(choice))) = true;
        end
    end
end
end
