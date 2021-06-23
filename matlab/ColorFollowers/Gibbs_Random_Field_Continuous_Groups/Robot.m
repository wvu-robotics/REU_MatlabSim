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
        goalX = 1;
        goalY = 1;
        color = 1;
        VMax = 0.5;
        sigma = 0.8;
        stepSize = 2;
        goalPotentialExp = 1.2;
        goalAllowance = 8;
        RGB = [0 0 0];
        
        %All just random values for now, need to be tuned
        %------------------------------------------------
%         charge = 4;
%         maxChargeProduct = 30;
%         obs_charge = 0.1;
%         epsilon = 0.1;
%         epsilon_not = 0.1;
%         alpha = 2;
%         r_not = 8;
%         mass = 0.05;
        
        charge = 4;
        maxChargeProduct = 30;
        obs_charge = 0.1;
        epsilon = 0.05;
        epsilon_not = 0.1;
        alpha = 4;
        r_not = 10;
        mass = 0.05;
        %------------------------------------------------
        
        neighborList;
        sensingRange = 5;
        timeStep;
    end
    
    methods
        function obj = Robot(id,x,y, ts)
            %ROBOT Construct an instance of this class
            obj.x = x; %init x coord
            obj.y = y;
            obj.truex = x;
            obj.truey = y;
            obj.id = id; %A unique ID for the robot
            
            obj.timeStep = ts;
        end
        
        
        function xy = get_xy(obj)
            %Returns a tuple of the robots present X and Y coords
            xy = [obj.x,obj.y];
        end
        
        %Sets the agent's goal to a random XY Position in the map
        function newGoal(obj, worldObj, type)
            switch type
                case 'random'
                    obj.goalX = randi(worldObj.max_x);
                    obj.goalY = randi(worldObj.max_y);
                case 'topRight'
                    obj.goalX = randi([round(worldObj.max_x*3/4),worldObj.max_x]);
                    obj.goalY = randi([round(worldObj.max_y*3/4),worldObj.max_y]);
                case 'topLeft'
                    obj.goalX = randi(round(worldObj.max_x/4));
                    obj.goalY = randi([round(worldObj.max_y*3/4),worldObj.max_y]);
                case 'bottomLeft'
                    obj.goalX = randi(round(worldObj.max_x/4));
                    obj.goalY = randi(round(worldObj.max_y/4));
            end
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
            if obj.x < 1
                obj.x = 1;
                obj.truex = 1;
                obj.Vx = 0;
                obj.Vy = 0;
            end
            if obj.y < 1
                obj.y = 1;
                obj.truey = 1;
                obj.Vx = 0;
                obj.Vy = 0;
            end
            if obj.x > worldObj.max_x
                obj.x = worldObj.max_x;
                obj.truex = worldObj.max_x;
                obj.Vx = 0;
                obj.Vy = 0;
            end
            if obj.y > worldObj.max_y
                obj.y = worldObj.max_y;
                obj.truey = worldObj.max_y;
                obj.Vx = 0;
                obj.Vy = 0;
            end
            xy = [obj.x,obj.y];
        end
        
        %This function find the indexes of all the neighbors of the central
        %agent and puts them in neighborList
        function getNeighbors(obj, worldObj)
            obj.neighborList = [];
            
            for i = 1:length(worldObj.robot_list)
               relativeX = worldObj.robot_list(i).x - obj.x;
               relativeY = worldObj.robot_list(i).y - obj.y;
               distance = norm([relativeX,relativeY]);
               if distance < obj.sensingRange && worldObj.robot_list(i).id ~= obj.id
                  %obj.neighborList = [obj.neighborList, worldObj.robot_list(i).id];
                  obj.neighborList = [obj.neighborList, i];
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
                
                %This is the old way with discrete color groups
%                 if obj.color == neighborRobot.color
%                     chargeProduct = -abs(obj.charge*neighborRobot.charge);
%                 else
%                     chargeProduct =  abs(obj.charge*neighborRobot.charge);
%                 end

                %This is the new way with continuous groups
                angularGoalDistToNeighbor = abs(neighborRobot.color - obj.color);
                if angularGoalDistToNeighbor > pi
                    angularGoalDistToNeighbor = 2*pi - angularGoalDistToNeighbor;
                end
                %If angular diference is pi, this is +maxChargeProduct. If
                %the angular difference is 0, this is -maxChargeProduct
                chargeProduct = angularGoalDistToNeighbor/pi * 2 * obj.maxChargeProduct - obj.maxChargeProduct;
                
                CBPotential = CBPotential + (obj.epsilon * (6/(obj.alpha - 6) * 2.718^(obj.alpha)...
                    * (1 - radius/obj.r_not) - obj.alpha/(obj.alpha - 6) * (obj.r_not/radius)^6)...
                    + chargeProduct/(4*pi*obj.epsilon_not*radius));
            end
            
            closeObstacles = obj.ObstacleAvoidingPotential(worldObj, propVel);
            OAPotential = 0;
            for i = 1:size(closeObstacles,1)
                radius = norm([propX, propY] - closeObstacles(i,:));
                chargeProduct = abs(obj.charge * obj.obs_charge);
                OAPotential = OAPotential + (obj.epsilon * (6/(obj.alpha - 6) * 2.718^(obj.alpha)...
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
            KESelf = 0.5 * obj.mass * (obj.VMax - norm(propVel))^2;
        end
        
        %This function find the list of obstacles that are close to the
        %central agent for use in the Coulomb buckingham function
        function closeObstacles = ObstacleAvoidingPotential(obj, worldObj, propVel)
            
            %make set of points on the walls
            tempWall = ones(length(1:obj.stepSize:worldObj.max_x),1)*worldObj.max_y;
            topWall = [(1:obj.stepSize:worldObj.max_x)', tempWall];
            tempWall = zeros(length(1:obj.stepSize:worldObj.max_x),1);
            bottomWall = [(1:obj.stepSize:worldObj.max_x)', tempWall];
            tempWall = ones(length(1:obj.stepSize:worldObj.max_y),1)*worldObj.max_x;
            rightWall = [tempWall, (1:obj.stepSize:worldObj.max_y)'];
            tempWall = zeros(length(1:obj.stepSize:worldObj.max_y),1);
            leftWall = [tempWall, (1:obj.stepSize:worldObj.max_y)'];
            
            %Make a list of close obstacles and check if the agent can see
            %any of the walls
            closeObstacles = [];
            if obj.truex - obj.sensingRange < 0
                theta = asin(obj.truex/obj.sensingRange);
                bottomBound = obj.truey - obj.sensingRange*cos(theta);
                topBound = obj.truey + obj.sensingRange*cos(theta);
                closeObstacles = [closeObstacles; leftWall(leftWall(:,2) > bottomBound & leftWall(:,2) < topBound,:)];
            elseif obj.truex + obj.sensingRange > worldObj.max_x
                theta = asin((worldObj.max_x - obj.truex)/obj.sensingRange);
                bottomBound = obj.truey - obj.sensingRange*cos(theta);
                topBound = obj.truey + obj.sensingRange*cos(theta);
                closeObstacles = [closeObstacles; rightWall(rightWall(:,2) > bottomBound & rightWall(:,2) < topBound,:)];
            end
            if obj.truey - obj.sensingRange < 0
                theta = asin(obj.truey/obj.sensingRange);
                bottomBound = obj.truex - obj.sensingRange*cos(theta);
                topBound = obj.truex + obj.sensingRange*cos(theta);
                closeObstacles = [closeObstacles; bottomWall(bottomWall(:,1) > bottomBound & bottomWall(:,1) < topBound,:)];
            elseif obj.truey + obj.sensingRange > worldObj.max_y
                theta = asin((worldObj.max_y - obj.truey)/obj.sensingRange);
                bottomBound = obj.truex - obj.sensingRange*cos(theta);
                topBound = obj.truex + obj.sensingRange*cos(theta);
                closeObstacles = [closeObstacles; topWall(topWall(:,1) > bottomBound & topWall(:,1) < topBound,:)];
            end
        end
        
        %This function calculates the probability of a given proposed
        %velocity given the potential functions and the agents around the
        %central agent
        function newVelocity = MetropolisHastings(obj, worldObj)
            mcmcChain = [obj.Vx, obj.Vy];
            [KENeighbor, KESelf] = obj.KineticEnergyPotential(worldObj, mcmcChain(1,:));
            [CBPotential, OAPotential] = obj.coulombBuckinghamPotential(worldObj, mcmcChain(1,:));
            goalPotential = GoalPotential(obj, mcmcChain(1,:));
            potentialChain = KENeighbor + KESelf + CBPotential + OAPotential + goalPotential;
            
            for i = 1:15
               
                propVx = normrnd(obj.Vx, obj.sigma);
                propVy = normrnd(obj.Vy, obj.sigma);
                [KENeighbor, KESelf] = obj.KineticEnergyPotential(worldObj, [propVx,propVy]);
                [CBPotential, OAPotential] = obj.coulombBuckinghamPotential(worldObj, [propVx,propVy]);
                goalPotential = GoalPotential(obj, [propVx,propVy]);
                newPotential = KENeighbor + KESelf + CBPotential + OAPotential + goalPotential;
                
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
        
        %This finds the potential generated from being far from a goal
        %point. The potential field is just linear
        function goalPotential = GoalPotential(obj, propVel)
            [propX, propY] = kinematicModel(obj, propVel);
            distToGoal = norm([obj.goalX - propX, obj.goalY - propY]);
            goalPotential = distToGoal^obj.goalPotentialExp;
        end
        
        %This checks if the agent is close enough to their goal to be
        %reassigned or deleted
        function closeEnough = CheckGoalAllowance(obj)
            distToGoal = norm([obj.goalX - obj.truex, obj.goalY - obj.truey]);
            if distToGoal < obj.goalAllowance
               closeEnough = true; 
            else
               closeEnough = false; 
            end
        end
        
        %This function find the vector to the goal and sets the agents
        %color to that vector. It also finds the new RGB for that agent and
        %sets it
        function FindNewColor(obj)
            vectToGoal = [obj.goalX, obj.goalY] - [obj.truex, obj.truey];
            newColor = mod(atan2(vectToGoal(2),vectToGoal(1)),2*pi);
            obj.color = newColor;
            obj.RGB(1) = ((newColor < pi).*(255-newColor*255/pi)) + ((newColor >= pi).*(newColor-pi)*255/pi);
            obj.RGB(2) = (and(newColor <= 5/3*pi,newColor >= 2/3*pi).*(255-(newColor-2/3*pi)*255/pi)) + ...
                ((newColor > 5/3*pi).*(newColor-5/3*pi)*255/pi) + ((newColor < 2/3*pi).*((newColor+1/3*pi)*255/pi));
            obj.RGB(3) = (and(newColor <= 4/3*pi,newColor >= 1/3*pi).*(newColor-1/3*pi)*255/pi) + ...
                ((newColor < 1/3*pi).*(255-(newColor+2/3*pi)*255/pi)) + ((newColor > 4/3*pi).*(255-(newColor-4/3*pi)*255/pi));
            
            colorSum = obj.RGB(1) + obj.RGB(2) + obj.RGB(3);
            obj.RGB = obj.RGB./(colorSum)*1.5;
        end
        
        
    end
end









