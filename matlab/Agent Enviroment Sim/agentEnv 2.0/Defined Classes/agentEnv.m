classdef agentEnv < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
     properties (Access = public)
        agents = Agent.empty;
        obstacles = staticObstacle.empty;
        realTime = true;
        collisions = 0;
     end
    
     properties (Access = private)
          
        shapeGroups;
        configurationSpace = polyshape.empty;
        collisionListPoseMin;
        collisionListPoseMax;
        collisionListID = string.empty;
        numberOfAgents;
        mapSize;
        timeStep; 
        figPS;
        lineAgent = patch;
        textAgentNumber = text;
        linePath = patch;
        obstaclePatch = patch;
        lineGoalLocations = line;
        goalLocations;
        isVisible = true;
        hasCollsions = true;
        angleArray = 0:(2*pi/128):2*pi;
        needsUpdate = false; 
        playbackFlag = false;
     end
    
    methods
%Constructor
        function obj = agentEnv(numberOfAgents, controller ,mapSize, timeStep)
            obj.numberOfAgents = numberOfAgents;
            obj.mapSize = mapSize;
            obj.timeStep = timeStep;
            obj.figPS = figure('Name','Position Space');
            axis([-mapSize mapSize -mapSize mapSize]);
            obj.lineGoalLocations = line('Marker', '*', ...
                                   'LineStyle', 'none', ...
                                   'Color', [1 0 0]);
            for i = 1:numberOfAgents
                obj.linePath(i) = patch('EdgeColor','interp','MarkerFaceColor','flat');
            end
            for i = 1:numberOfAgents
                obj.agents(i) = Agent(i,timeStep);
                obj.lineAgent(i) = patch('lineWidth', 1, 'facecolor', 'none');
                obj.textAgentNumber(i) = text;
                
                set(obj.textAgentNumber(i), 'String', i)
                set(obj.lineAgent(i),'edgecolor', 'b')
            end
            if length(controller) == 1
                if isa(controller,'cell')
                    controller = cell2mat(controller);
                end
                for i = 1:numberOfAgents
                    obj.agents(i).setController(controller);
                end 
            else
                for i = 1:numberOfAgents
                    obj.agents(i).setController(cell2mat(controller(i)));
                end
            end
        end
% Setters
        function setAgentPositions(obj,pose)
            for i = 1:obj.numberOfAgents
                obj.agents(i).pose = pose(i,:);
                obj.updateAgentPath(i,pose(i,:));
            end 
        end
        
        function setAgentVelocities(obj, vel)
            for i = 1:obj.numberOfAgents
                obj.agents(i).velocity  = vel(i,:);
            end
        end
        
        function setGoalPositions(obj,goalPose)
            for i = 1:obj.numberOfAgents
                obj.agents(i).goalPose = goalPose(i,:);
            end 
            obj.goalLocations = goalPose;
        end
        
        function setAgentColor(obj,id,color)
            obj.agents(id).color = color;
            set(obj.lineAgent(id), 'edgecolor', color);
        end
        
%getters       
        function pose = getAgentPositions(obj)
            pose = zeros(obj.numberOfAgents, 2); 
            for i = 1:obj.numberOfAgents
                pose(i,:) = obj.agents(i).pose; 
            end
        end
        
        function vel = getAgentVelocities(obj)
            vel = zeros(obj.numberOfAgents, 2); 
            for i = 1:obj.numberOfAgents
                vel(i,:) = obj.agents(i).velocity; 
            end
        end
        
        function numAgents = getNumberOfAgents(obj)
            numAgents = obj.numberOfAgents;
        end 
        
%functionaility
        function physics(obj, id)

                obj.agents(id).heading = obj.angleArray(obj.convertToDiscAngle(obj.agents(id).heading));
                obj.agents(id).pose = obj.findAgentControllerKinematics(id);
                hasCollided = false;
                if obj.hasCollsions
                    obj.agentCollider(id);
                    if hasCollided
                        obj.collisions = obj.collisions + 1;
                    end
                end
                obj.agents(id).velocity = (obj.agents(id).pose - obj.agents(id).path(end,:))/obj.timeStep;
                obj.updateAgentPath(id,obj.agents(id).pose);
        end
        
        function updateAgentColor(obj,id)
            obj.lineAgent(id).EdgeColor = obj.agents(id).color;
        end
        
        function agentCollider(obj,id)
            obj.updateCollisionList('A',id);   
            potentialCollsionIndex = obj.findPotentialCollisions(id, 'A'); 
            if ~isempty(potentialCollsionIndex)
                hasCollided = obj.detectCollision(id,potentialCollsionIndex);
                if hasCollided
                    collisionResolved = false;
                    converged = false;
                    initialHeading = 0;
                    obj.agents(id).heading = obj.agents(id).previousHeading(end);
                    desiredHeading = obj.agents(id).heading - obj.agents(id).previousHeading(end);
                    initialPose = 0;
                    desiredPose = norm(obj.agents(id).pose - obj.agents(id).path(end,:));
                    desiredPoseUnitVec = obj.agents(id).pose - obj.agents(id).path(end,:);
                    desiredPoseUnitVec = desiredPoseUnitVec./norm(desiredPoseUnitVec);
                    newRelativePose = [];
                
                    while ~collisionResolved
 
                        if ~isempty(newRelativePose)
                            if abs(desiredPose - initialPose) < .001 
                                desiredPose = initialPose;
                                desiredHeading = initialHeading;
                            end
                        end

                        newRelativePose = abs(desiredPose - initialPose)/2 + initialPose;
                        newHeading = (desiredHeading - initialHeading)/2 + initialHeading;

                    
                        obj.agents(id).heading = obj.angleArray(obj.convertToDiscAngle(newHeading + obj.agents(id).previousHeading(end)));
                        obj.agents(id).pose = newRelativePose.*desiredPoseUnitVec + obj.agents(id).path(end,:);
                    
                        obj.updateCollisionList('A',id);   
                        potentialCollsionIndex = obj.findPotentialCollisions(id, 'A');  
                        if obj.detectCollision(id,potentialCollsionIndex) & any(newRelativePose)
                            if round(newRelativePose,5) == round(desiredPose,5)
                                converged = true;
                            else
                                desiredPose = newRelativePose;
                                desiredHeading = newHeading;    
                            end
                
                            elseif abs(desiredPose - initialPose) <= .05
                                collisionResolved = true;      
                            else
                            if round(newRelativePose,5) == round(initialPose,5)
                                converged = true;
                            else
                            initialPose = newRelativePose;
                            initialHeading = newHeading;
                            end
                        end
                    end
                end
            end
        end
        
        function potentialCollsionIndex = findPotentialCollisions(obj, id, type)
            index = find( obj.collisionListID == strcat(type,string(id)));
            potentialCollsionIndex = unique([find(obj.collisionListPoseMin >= obj.collisionListPoseMin(index) & ...
                                                  obj.collisionListPoseMin <= obj.collisionListPoseMax(index)), ...
                                             find(obj.collisionListPoseMax >= obj.collisionListPoseMin(index) & ...
                                                  obj.collisionListPoseMax <= obj.collisionListPoseMax(index)), ...
                                             find(obj.collisionListPoseMin <= obj.collisionListPoseMin(index) & ...
                                                  obj.collisionListPoseMax >= obj.collisionListPoseMax(index))]);
             agentIndex = find(obj.collisionListID == append(type,num2str(id)));
             potentialCollsionIndex = potentialCollsionIndex(potentialCollsionIndex ~= agentIndex);
        end
        
        function hasCollided = detectCollision(obj, id, potentialCollsionIndex)
             hasCollided = false; 
             for i = potentialCollsionIndex
                potentialCollisonID = char(obj.collisionListID(i));
                if potentialCollisonID(1) == 'A' 
                    agentsType = obj.agents;
                else
                    agentsType = obj.obstacles;
                end
                    obstacleID = str2num(potentialCollisonID(2:end));
                    if ~(obstacleID == id  & potentialCollisonID(1) == 'A') 
                        relativeHeading = obj.agents(id).heading-agentsType(obstacleID).heading;
                    
                        configurationSpaceIndex = obj.convertToDiscAngle(relativeHeading);
                    
                        obj.agents(id).heading = obj.angleArray(obj.convertToDiscAngle(obj.angleArray(configurationSpaceIndex)+ agentsType(obstacleID).heading));
                        configurationSpacePolygon = obj.configurationSpace(configurationSpaceIndex,obj.agents(id).getShapeID, agentsType(obstacleID).getShapeID).Vertices;
                        newShape = obj.transformShape(configurationSpacePolygon,agentsType(obstacleID).heading);
                        newShape(1,:) = newShape(1,:) + agentsType(obstacleID).pose(1);
                        newShape(2,:) = newShape(2,:) + agentsType(obstacleID).pose(2);
                    
                        [collision]= inpolygon( obj.agents(id).pose(1) ,obj.agents(id).pose(2) , ...
                                                        newShape(1,:), ...
                                                        newShape(2,:));
                        if collision             
                             hasCollided = true;
                             break
                        end   
                    end
             end 
        end
        
        function configurationSpaceIndex = convertToDiscAngle(obj,inputHeading)

            inputHeading = wrapTo2Pi(inputHeading);
            if any(inputHeading)
               [~,configurationSpaceIndex ] = min(abs(obj.angleArray - inputHeading));
            else
               configurationSpaceIndex = 1;
            end
        end
        
        function createStaticObstacle(obj,shape, pose, heading, id)
            obj.obstacles(id) =  staticObstacle(shape, pose, heading, id);
            shape = obj.transformShape(shape,heading)';
            obj.obstaclePatch(id) = patch('XData', shape(:,1) + pose(1)*ones(length(shape(:,1)),1), ...
                                          'YData', shape(:,2) + pose(2)*ones(length(shape(:,1)),1), ...
                                          'FaceColor',[.5 .5 .5],... 
                                          'edgeColor', 'none');
            obj.updateCollisionList('S',length(obj.obstacles));
        end
              
        function pathVisibility(obj, isVisible)
            for i = 1:obj.numberOfAgents
                set(obj.linePath(i),'visible',isVisible)
            end
            obj.isVisible = isVisible;
        end
        
        function agentIdVisibility(obj, isVisible)
            for i = 1:obj.numberOfAgents
                set(obj.textAgentNumber(i),'visible',isVisible)
            end
        end
        
        function collisionsOn(obj, hasCollsion)
            obj.hasCollsions = hasCollsion;
        end
            
        function updateAgentPath(obj,agent,pose)
                if isempty(obj.agents(agent).path)
                    obj.agents(agent).path(1,:) = pose;
                    obj.agents(agent).pathColor(1,:) = obj.agents(agent).color;
                    obj.agents(agent).previousHeading(1,:) = obj.agents(agent).heading;
                else
                    obj.agents(agent).path(length(obj.agents(agent).path(:,1))+1,:) = pose;
                    obj.agents(agent).pathColor(length(obj.agents(agent).pathColor(:,1))+1,:) = obj.agents(agent).color;
                    obj.agents(agent).previousHeading(length(obj.agents(agent).previousHeading(:,1))+1,:) = obj.agents(agent).heading;
                end       
        end
        
        function updateAgentKinematics(obj,id)
               obj.agents(id).pose = obj.agents(id).pose + obj.agents(id).velocity*obj.timeStep;
        end
        
        function newPose = findAgentControllerKinematics(obj, id)
               newPose = obj.agents(id).pose + obj.agents(id).velocityControl*obj.timeStep;
        end

        function updateGraph(obj)
            obj.lineGoalLocations.XData =  obj.goalLocations(:,1);
            obj.lineGoalLocations.YData = obj.goalLocations(:,2);
            for i = 1:obj.numberOfAgents
                obj.updateAgentColor(i);
                obj.drawAgent(i);
                obj.textAgentNumber(i).Position = [obj.agents(i).pose(1)  ...
                                                   obj.agents(i).pose(2)];                                
                   
                 if ~obj.playbackFlag && obj.isVisible             
                        obj.linePath(i).XData = obj.agents(i).path(:,1);
                        obj.linePath(i).YData = [obj.agents(i).path(1:(end-1),2); NaN];
                        obj.linePath(i).FaceVertexCData = obj.agents(i).pathColor;
                 end
                    
            end
        end
        
        function drawAgent(obj,id)
            heading = obj.agents(id).heading;
            shape = obj.agents(id).getShape;
            newShape = obj.transformShape(shape, heading);  
            x = newShape(1,:) + obj.agents(id).pose(1);
            y = newShape(2,:) + obj.agents(id).pose(2);
            set(obj.lineAgent(id),'xdata',x,'ydata',y)
        end

        function newShape = transformShape(~,shape, heading)
            newShape =[cos(heading) sin(heading);
                      -sin(heading) cos(heading)]*[shape(:,1)';shape(:,2)'];
        end
        
        function newPose = transform2D(~,pose,heading)
            newPose =[cos(heading) sin(heading);
                      -sin(heading) cos(heading)]*pose;
        end 
           
        function updateCollisionList(obj, type, id)
            objectID = append(type,string(id));
            if ~ismember(objectID,obj.collisionListID) 
               obj.collisionListID(end + 1) = objectID;
            end
            index = find(obj.collisionListID == objectID );
            if type == 'A'
                  shape = obj.transformShape(obj.agents(id).broadCollisionSpace,obj.agents(id).heading);
                  obj.collisionListPoseMin(index) = min(shape(1,:)) + obj.agents(id).pose(1);
                  obj.collisionListPoseMax(index) = max(shape(1,:)) + obj.agents(id).pose(1);
            elseif type == 'S'
                  shape = obj.transformShape(obj.obstacles(id).broadCollisionSpace,obj.obstacles(id).heading);
                  obj.collisionListPoseMin(index) = min(shape(1,:)) + obj.obstacles(id).pose(1);
                  obj.collisionListPoseMax(index) = max(shape(1,:)) + obj.obstacles(id).pose(1);
            end
            
            [obj.collisionListPoseMin,newIndexs] = sort(obj.collisionListPoseMin);
            newListID = string.empty;
            for i = 1:length(obj.collisionListPoseMin)
               if newIndexs(i) ~= i
                   newListID(i) = obj.collisionListID(newIndexs(i));
                   newMaxList(i) = obj.collisionListPoseMax(newIndexs(i));
               else
                   newListID(i) = obj.collisionListID(i);
                   newMaxList(i) = obj.collisionListPoseMax(i);
               end
            end
            obj.collisionListID = newListID;
            obj.collisionListPoseMax = newMaxList;
        end
        
        function createConfigurationSpace(obj, obj1, obj2)
%           obj.configurationSpace(1:length(1:.1:2*pi),obj2.getShapeID, obj1.getShapeID ) = cell(length(1:.1:2*pi),1,1);
            index = 1;
            for i = obj.angleArray
                shape1 = obj1.getShape;
                shape2 = obj2.getShape;
                %shape1 = obj.transformShape(shape1, i);
                shape2 = obj.transformShape(shape2, i);
                S = MinkDiff(shape1,shape2');
%                 S(:,1) = -S(:,1);
                pgon = polyshape(S(:,1),S(:,2), 'Simplify', false);
                obj.configurationSpace(index, obj2.getShapeID,obj1.getShapeID ) = convhull(pgon);
                index = index + 1;
            end
        end
        
        function detectShapes(obj)           
            obj.shapeGroups(1,1,1) = 1;
            if ~isempty(obj.obstacles)
                indexType = [1,2];
                obj.shapeGroups(1,1,2) = 1;
            else
                indexType = 1;
                obj.shapeGroups(1,1,2) = 0;
            end
            
            for type = indexType
                if type == 1
                    obstacleArray = obj.agents;
                    obstacleArray(1).setShapeID(1);
                    groupNumberOffset = 0;
                elseif type == 2
                    obstacleArray = obj.obstacles;
                    obstacleArray(1).setShapeID(groupNumberOffset+1);        
                end
                 numberOfObstacles = length(obstacleArray);
                undefinedGroups = [];
                for i = 2:numberOfObstacles
                        undefinedGroups(i-1) = obstacleArray(i).getID;
                end
                agentShape = obstacleArray(1).getShape; 
                numberOfMembers = 2;
                groupNumber = 1;
                while ~isempty(undefinedGroups)
                newUndefinedGroups = [];
                    for i = 1:length(undefinedGroups)
                        if size(agentShape) == size(obstacleArray(undefinedGroups(i)).getShape)
                            if agentShape == obstacleArray(undefinedGroups(i)).getShape
                                obj.shapeGroups(groupNumber, numberOfMembers, type) = obstacleArray(undefinedGroups(i)).getID;
                                obstacleArray(undefinedGroups(i)).setShapeID(groupNumber + groupNumberOffset);  
                                numberOfMembers = numberOfMembers + 1;
                            else
                                newUndefinedGroups(end + 1) = obstacleArray(undefinedGroups(i)).getID; 
                            end
                        else
                            newUndefinedGroups(end + 1) = obstacleArray(undefinedGroups(i)).getID; 
                        end
                    end
                        undefinedGroups = newUndefinedGroups;
                    if ~isempty(undefinedGroups)
                        agentShape = obstacleArray(undefinedGroups(1)).getShape;
                        groupNumber = groupNumber + 1; 
                        numberOfMembers = 1; 
                        if type == 1
                            obj.agents = obstacleArray;
                        elseif type == 2
                            obj.obstacles = obstacleArray;
                        end
                    end                
                end  
                groupNumberOffset = groupNumber;
            end    
        end
        
        function updateConfigurationSpace(obj)
            numberOfGroups = (nnz(obj.shapeGroups(:,1,1)) + ...
                              nnz(obj.shapeGroups(:,1,2)));
            for i = 1:numberOfGroups
                if i <= nnz(obj.shapeGroups(:,1,1))
                   obstacleType1 = obj.agents;
                   typeIndex1 = 1;
                   diff1 = 0;
                else
                   obstacleType1 = obj.obstacles;
                   typeIndex1 = 2;
                   diff1 = nnz(obj.shapeGroups(:,1,1));
                end
                
                for j = 1:numberOfGroups
                    if j <= nnz(obj.shapeGroups(:,1,1))
                        obstacleType2 = obj.agents;
                        typeIndex2 = 1;
                        diff2 = 0;
                    else
                        obstacleType2 = obj.obstacles;
                        typeIndex2 = 2;
                        diff2 = nnz(obj.shapeGroups(:,1,1));
                    end
                      obj.createConfigurationSpace(obstacleType1(obj.shapeGroups(i - diff1,1,typeIndex1)), ...
                                                   obstacleType2(obj.shapeGroups(j - diff2,1,typeIndex2)))
                end
            end 
        end
        
        function updateEnv(obj)
            obj.detectShapes;
            obj.updateConfigurationSpace;
        end
        
        function checkIfUpdateIsNeeded(obj)
            if obj.needsUpdate == true
                obj.updateEnv;
                obj.needsUpdate = false;
                for i = 1:obj.numberOfAgents
                   obj.agents(i).needsUpdate = false; 
                end
            else
                for i = 1:obj.numberOfAgents
                    if obj.agents(i).needsUpdate == true
                        obj.updateEnv;
                        obj.needsUpdate = false;
                        for j = 1:obj.numberOfAgents
                            obj.agents(j).needsUpdate = false; 
                        end
                        break
                    end
                end
            end
        end
        
        function tick(obj)
            obj.checkIfUpdateIsNeeded;
            tStart = cputime;
            
            %Gets measurements and velocity controls for each agent
            for i = 1:obj.numberOfAgents
                obj.agents(i).callMeasurement(obj);
                obj.agents(i).callController;
            end
            
            %Handles kinematics and collision simulation for all agents
            for i = randperm(obj.numberOfAgents)
                obj.updateCollisionList('A',i);
                obj.physics(i);
            end
            
            obj.updateGraph; %Can be put inside for loop but slower
            
            tEnd = cputime - tStart;
            if obj.realTime
                if tEnd > obj.timeStep
                    pause(.0001);
                else
                    pause(obj.timeStep - tEnd);
                end
            else
                pause(.001);
            end
            
        end 
        
        function playback(obj, path)
            if exist(path, 'dir')
                writerObj = VideoWriter(append(path,'playbackResult.avi'));
                obj.playbackFlag =true;
                obj.hasCollsions = false;

                sampleFrequency = 1/obj.timeStep;
                if sampleFrequency > 30
                    sampleFrequency = 30;
                end
                writerObj.FrameRate = sampleFrequency;
                writerObj.Quality = 75;
                open(writerObj);
                counter = 0;
                counterTimeStep = 0;
                for i = 1:length(obj.agents(1).path(:,1))
                
                    for j = 1:length(obj.agents) 
                        obj.agents(j).pose = obj.agents(j).path(i,:);
                        obj.agents(j).heading = obj.agents(j).previousHeading(i);
                        obj.agents(j).color = obj.agents(j).pathColor(i,:);
                        if obj.isVisible
                            obj.linePath(j).XData = obj.agents(j).path(1:i,1);
                            obj.linePath(j).YData = [obj.agents(j).path(1:(i-1),2); NaN];
                            obj.linePath(j).FaceVertexCData = obj.agents(j).pathColor(1:i,:);
                        end
                    end 
                
                    obj.updateGraph;
            
                    if sampleFrequency == 30
                        if counterTimeStep >= (counter/30)
                            writeVideo(writerObj,getframe(obj.figPS));
                            counter = counter +1;
                        end
                    else
                        writeVideo(writerObj,getframe(obj.figPS))
                    end
                    counterTimeStep = counterTimeStep + obj.timeStep;
                    pause(.001)
                end
                close(writerObj);
            else
                disp('ERROR: Invalid path');
            end
        end
        
        %ros functionality
        function tickRos(obj)
            tStart = cputime;
            for i = randperm(obj.numberOfAgents)
                obj.agents(i).msgSub = receive(obj.agents(i).subscriber);
                
                obj.agents(i).pose = [obj.agents(i).msgSub.Transform.Translation.X, ...
                                      obj.agents(i).msgSub.Transform.Translation.Y]; 
                eulAngles = quat2eul([obj.agents(i).msgSub.Transform.Rotation.X, ...
                             obj.agents(i).msgSub.Transform.Rotation.Y, ...
                             obj.agents(i).msgSub.Transform.Rotation.Z, ...
                             obj.agents(i).msgSub.Transform.Rotation.W]);
                         
                obj.agents(i).heading = eulAngles(3); 
                obj.agents(i).callMeasurement(obj);          
                obj.agents(i).callController;
                contVel = obj.transform2D([obj.agents(i).velocityControl(1);
                                           obj.agents(i).velocityControl(2)], ...
                                           obj.agents(i).heading);
                               
                obj.agents(i).msgPub.Linear.X = contVel(1);
                obj.agents(i).msgPub.Linear.Y = contVel(2);
                obj.agents(i).msgPub.Angular.Z = obj.agents(i).angularVelocityControl;
                
                obj.agents(i).heading = obj.agents(i).heading + obj.agents(i).angularVelocityControl*obj.timeStep;
                obj.agents(i).pose = obj.findAgentControllerKinematics(i);
                  send(obj.agents(i).publisher,obj.agents(i).msgPub);
                obj.updateAgentPath(i,obj.agents(i).pose);
            end
            obj.updateGraph;
            tEnd = cputime - tStart;
            if obj.realTime
                if tEnd > obj.timeStep
                    pause(.0001);
                else
                    pause(obj.timeStep - tEnd);
                end
            else
                pause(.001);
            end
            
        end
    end
end





