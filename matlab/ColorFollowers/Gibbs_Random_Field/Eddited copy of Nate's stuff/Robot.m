classdef Robot < handle
    %ROBOT Summary of this class goes here
    %One robot object for each simulated robot. The robot decides how it
    %will move and returns it's actions to the owning class.
    properties
        id = 0;
        x = 1;
        y = 1;
        truex = 1;
        truey = 1;
        Vx = 0;
        Vy = 0;
        color;
        VMax = 1.5;
        sigma = 0.4;
        
        %All just random values for now, need to be tuned
        %------------------------------------------------
        charge = 4;
        obs_charge = 4;
        epsilon = 0.04;
        epsilon_not = 0.04;
        alpha = 1;
        r_not = 5;
        mass = 0.05;
        %------------------------------------------------
        
        local_map;
        neighborList;
        sensingRange = 5;
        timeStep;
    end
    
    methods
        function obj = Robot(id,x,y, ts, col, map_in)
            %ROBOT Construct an instance of this class
            obj.x = x; %init x coord
            obj.y = y;
            obj.truex = x;
            obj.truey = y;
            obj.id = id; %A unique ID for the robot
            obj.color = col;
            
            obj.local_map = map_in;
            obj.timeStep = ts;
        end
        
        
        function xy = get_xy(obj)
            %Returns a tuple of the robots present X and Y coords
            xy = [obj.x,obj.y];
        end

        %This takes the newVelocity output by the grf method and uses it to
        %find the next square to go to, considering walls and other agents
        %but not obstacles yet
        function xy = move(obj, worldObj)
            obj.getNeighbors(worldObj);
            newVelocity = MetropolisHastings(obj, worldObj);
            obj.Vx = newVelocity(1);
            obj.Vy = newVelocity(2);
            
            [obj.truex, obj.truey] = obj.kinematicModel(newVelocity);
            obj.x = round(obj.truex);
            obj.y = round(obj.truey);
            xy = [obj.x,obj.y];
            
%             forceAngle = mod(atan2(newVelocity(2), newVelocity(1)),2*pi);
%             
%             xBottomWall = false;
%             yBottomWall = false;
%             xTopWall = false;
%             yTopWall = false;
%             if obj.x == 1
%                 xBottomWall = true;
%             end
%             if obj.y == 1
%                 yBottomWall = true;
%             end
%             if obj.x == worldObj.max_x
%                 xTopWall = true;
%             end
%             if obj.y == worldObj.max_y
%                 yTopWall = true;
%             end
%             found = false;
%             
%             movePlan = [0, 0, 0, 0, 0];
%             if forceAngle <= pi/8 || forceAngle > 15/8*pi
%                 movePlan = [1, 8, 2, 7, 3]; %Cell 1
%             elseif forceAngle <=3/8*pi
%                 movePlan = [2, 1, 3, 8, 4]; %Cell 2
%             elseif forceAngle <=5/8*pi
%                 movePlan = [3, 2, 4, 1, 5]; %Cell 3
%             elseif forceAngle <=7/8*pi
%                 movePlan = [4, 3, 5, 2, 6]; %Cell 4
%             elseif forceAngle <=9/8*pi
%                 movePlan = [5, 4, 6, 3, 7]; %Cell 5
%             elseif forceAngle <=11/8*pi
%                 movePlan = [6, 5, 7, 4, 8]; %Cell 6
%             elseif forceAngle <=13/8*pi
%                 movePlan = [7, 6, 8, 5, 1]; %Cell 7
%             else
%                 movePlan = [8, 7, 1, 6, 2]; %Cell 8
%             end
%             
%             move = [1, 1, 0, -1, -1, -1, 0, 1; 0, 1, 1, 1, 0, -1, -1, -1]; %First row add to X, second row add to Y for each move plan
%             xy = [obj.x,obj.y];
%             for choice = 1:length(movePlan)
%                 if ((move(1, movePlan(choice)) == 1 && not(xTopWall)) || (move(1, movePlan(choice)) == -1 && not(xBottomWall)) || (move(1, movePlan(choice)) == 0))...
%                         && ((move(2, movePlan(choice)) == 1 && not(yTopWall)) || (move(2, movePlan(choice)) == -1 && not(yBottomWall)) || (move(2, movePlan(choice)) == 0))...
%                         && not(found) && worldObj.worldMap.map(obj.x+move(1, movePlan(choice)), obj.y+move(2, movePlan(choice)),worldObj.robot_layer) == 0 ...
%                         && worldObj.worldMap.map(obj.x+move(1, movePlan(choice)), obj.y+move(2, movePlan(choice)),worldObj.obs_layer) == 0
%                     xy = [obj.x+move(1, movePlan(choice)), obj.y+move(2, movePlan(choice))];
%                     found = true;
%                 end
%             end
%             obj.x = xy(1);
%             obj.y = xy(2);
        end
        
        %This function find the indexes of all the neighbors of the central
        %agent and puts them in neighborList
        function getNeighbors(obj, worldObj)
            obj.neighborList = [];
            for i = obj.x-obj.sensingRange:obj.x+obj.sensingRange
                for j = obj.y-obj.sensingRange:obj.y+obj.sensingRange
                    if i > 0 && i < worldObj.max_x && j > 0 && j < worldObj.max_y
                        if worldObj.worldMap.map(i,j,worldObj.robot_layer) ~= 0
                            obj.neighborList(length(obj.neighborList)+1) = worldObj.worldMap.map(i,j,worldObj.robot_layer);
                        end
                    end
                end
            end
        end
        
        %Very Basic Kinematic Model
        function [x,y] = kinematicModel(obj, propVel)
            x = obj.truex + propVel(1) * obj.timeStep;
            y = obj.truey + propVel(2) * obj.timeStep;
        end
        
        %This function finds the sum of all the coulomb buckingham
        %potentials for all the neighbors for a given proposed velocity as
        %well as the CB potential from the static obstacles
        function [CBPotential, OAPotential] = coulombBuckinghamPotential(obj, worldObj, propVel)
            CBPotential = 0;
            [propX, propY] = obj.kinematicModel(propVel);
            
            for i = 1:length(obj.neighborList)
                neighborRobot = worldObj.robot_list(obj.neighborList(i));
                [neighX, neighY] = neighborRobot.kinematicModel([neighborRobot.Vx, neighborRobot.Vy]);
                radius = sqrt((propX-neighX)^2 + (propY-neighY)^2);
                
                if obj.color == neighborRobot.color
                    chargeProduct = -abs(obj.charge*neighborRobot.charge);
                else
                    chargeProduct =  abs(obj.charge*neighborRobot.charge);
                end
                
                CBPotential = CBPotential + (obj.epsilon * (6/(obj.alpha - 6) * exp(obj.alpha)...
                    * (1 - radius/obj.r_not) - obj.alpha/(obj.alpha - 6) * (obj.r_not/radius)^6)...
                    + chargeProduct/(4*pi*obj.epsilon_not*radius));
            end
            
            closeObstacles = obj.ObstacleAvoidingPotential(worldObj, propVel);
            OAPotential = 0;
            for i = 1:length(closeObstacles)
                radius = norm([propX, propY] - closeObstacles(i,:));
                chargeProduct = abs(obj.charge * obj.obs_charge);
                OAPotential = OAPotential + (obj.epsilon * (6/(obj.alpha - 6) * exp(obj.alpha)...
                    * (1 - radius/obj.r_not) - obj.alpha/(obj.alpha - 6) * (obj.r_not/radius)^6)...
                    + chargeProduct/(4*pi*obj.epsilon_not*radius));
            end
        end
        
        %This function finds the Kinetic Energy potential imposed by the
        %neighbors as well as the KE potential imposed by the self
        function [KENeighbor, KESelf] = KineticEnergyPotential(obj, worldObj, propVel)
            neighborObjList = worldObj.robot_list(obj.neighborList);
            
            VxSum = 0;
            VySum = 0;
            group_mass = 0;
            for i = 1:length(neighborObjList)
                if neighborObjList(i).color == obj.color
                    VxSum = VxSum + (neighborObjList(i).Vx - obj.Vx);
                    VySum = VySum + (neighborObjList(i).Vy - obj.Vy);
                    group_mass = group_mass + neighborObjList(i).mass;
                end
            end
            
            KENeighbor = 0.5 * group_mass * (VxSum^2 + VySum^2);
            KESelf = 0.5 * obj.mass * norm(obj.VMax - propVel)^2;
        end
        
        %This function find the list of obstacles that are close to the
        %central agent for use in the Coulomb buckingham function
        function closeObstacles = ObstacleAvoidingPotential(obj, worldObj, propVel)
            [obsXs, obsYs] = find(worldObj.worldMap.map(:,:,worldObj.obs_layer));
            obstacleLocations = [obsXs, obsYs];
            [propX,propY] = obj.kinematicModel(propVel);
            distToObstacles = vecnorm([obsXs, obsYs] - [propX,propY],2,2);
            closeObstacles = obstacleLocations(distToObstacles <= obj.sensingRange);
        end
        
        %This function calculates the probability of a given proposed
        %velocity given the potential functions and the agents around the
        %central agent
        function newVelocity = MetropolisHastings(obj, worldObj)
            mcmcChain = [obj.Vx, obj.Vy];
            [KENeighbor, KESelf] = obj.KineticEnergyPotential(worldObj, mcmcChain(1,:));
            [CBPotential, OAPotential] = obj.coulombBuckinghamPotential(worldObj, mcmcChain(1,:));
            potentialChain = KENeighbor + KESelf + CBPotential + OAPotential;
            
            for i = 1:100
               
                propVx = normrnd(obj.Vx, obj.sigma);
                propVy = normrnd(obj.Vy, obj.sigma);
                [KENeighbor, KESelf] = obj.KineticEnergyPotential(worldObj, [propVx,propVy]);
                [CBPotential, OAPotential] = obj.coulombBuckinghamPotential(worldObj, [propVx,propVy]);
                newPotential = KENeighbor + KESelf + CBPotential + OAPotential;
                
                dE = newPotential - potentialChain(length(potentialChain));
                grf = exp(-dE);
                
                if dE < 0 || grf > rand(1)
                    mcmcChain(size(mcmcChain,1)+1, :) = [propVx, propVy];
                    potentialChain(length(potentialChain)+1) = newPotential;
                else
                     mcmcChain(size(mcmcChain,1)+1, :) = mcmcChain(size(mcmcChain,1),:);
                     potentialChain(length(potentialChain)+1) = potentialChain(length(potentialChain));
                end
            end
            
            mcmcChain(1:floor(length(mcmcChain)/2),:) = [];
            newVelocity(1,1) = mean(mcmcChain(:,1));
            newVelocity(1,2) = mean(mcmcChain(:,2));
            
            if norm(newVelocity) > obj.VMax
               newVelocity = newVelocity/norm(newVelocity)*obj.VMax; 
            end
        end
        
    end
end









