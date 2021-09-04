%% findForceDirection(environment, unitToGoal, mapSize, sensingRadius, theta, maxNeighborToGoalForceRatio, middleVariance)
%   This function does the actual control for the Color_Followers_Local_Diversity script
%   by finding the appropriate centroid and cooresponding velocity control
%   for each agent.

function forceAngle = findForceDirection(environment, unitToGoal, mapSize, sensingRadius, theta, maxNeighborToGoalForceRatio, middleVariance)

%Set initial force to be the unit in the dirrection of the goal point for
%each agent and initialize the forceAngle matrix
force = unitToGoal;
forceAngle = zeros(mapSize, mapSize);

%Initialize centroid and relativeVentroidUnit matricies
relativeCentroid = zeros(mapSize, mapSize, 2); %x is front matrix, y in back
relativeCentroidUnit = zeros(mapSize, mapSize, 2);

%Go through every square in the map and see if there is an agent there. If
%there is, go through their entire sensing range and update the force for
%each other agent in the range
for i = 1:mapSize
    for j = 1:mapSize
        if environment.occupancy(i,j) == 1
            xCentroidNumer = 0;
            xCentroidDen = 0;
            yCentroidNumer = 0;
            yCentroidDen = 0;
            colorList = [];
            for k = -sensingRadius:sensingRadius %x offset in sensing range
                for l = -sensingRadius:sensingRadius %y offset in sensing range
                    if i+k > 0 && i+k <= mapSize && j+l > 0 && j+l <= mapSize && k ~= 0 && l ~= 0 && environment.occupancy(i+k, j+l) == 1 %make sure not off the edge of the map and not on central agent
                        
                        %Find 'angle' between colors
                        angularGoalDistToNeighbor = abs(theta(i+k, j+l) - theta(i, j));
                        if angularGoalDistToNeighbor > pi
                            angularGoalDistToNeighbor = 2*pi - angularGoalDistToNeighbor;
                        end
                        
                        %convert that 'angle' to an 'area' noramlized to 1
                        neighborArea = (pi - angularGoalDistToNeighbor)/pi;
                        
                        %Find necessary components to get the centroid
                        xCentroidNumer = xCentroidNumer + neighborArea * k;
                        yCentroidNumer = yCentroidNumer + neighborArea * l;
                        xCentroidDen = xCentroidDen + neighborArea;
                        yCentroidDen = yCentroidDen + neighborArea;
                        
                        %Add this agents 'color' to the colorList
                        colorList(length(colorList)+1) = theta(i+k, j+l);
                        
                    end 
                end
            end
            %Find centroid
            relativeCentroid(i,j,1) = xCentroidNumer/xCentroidDen;
            relativeCentroid(i,j,2) = yCentroidNumer/yCentroidDen;
            relativeCentroidUnit(i,j,1) = relativeCentroid(i,j,1)/norm([relativeCentroid(i,j,1),relativeCentroid(i,j,2)]);
            relativeCentroidUnit(i,j,2) = relativeCentroid(i,j,2)/norm([relativeCentroid(i,j,1),relativeCentroid(i,j,2)]);
            
            %Find variance of the colorList
            localColorVariance = var(colorList);
            
            %High variance should be negative maxNeighborToGoalForceRatio, low variance should be
            %positive maxNeighborToGoalForceRatio
            normalizedLocalColorVariance = (middleVariance - localColorVariance)/middleVariance * maxNeighborToGoalForceRatio;
            if normalizedLocalColorVariance > maxNeighborToGoalForceRatio
                normalizedLocalColorVariance = maxNeighborToGoalForceRatio;
            elseif normalizedLocalColorVariance < -maxNeighborToGoalForceRatio
                normalizedLocalColorVariance = -maxNeighborToGoalForceRatio;
            end
            
            %Add the force vector to the centroid to the goal force vector,
            %but scaled by the normalizedLocalColorVariance
            force(i,j, 1) = force(i,j,1) + normalizedLocalColorVariance * relativeCentroidUnit(i,j,1);
            force(i,j, 2) = force(i,j,2) + normalizedLocalColorVariance * relativeCentroidUnit(i,j,2);
            
            forceAngle(i,j) = mod(atan2(force(i,j,2),force(i,j,1)),2*pi);
        end
    end
end
end