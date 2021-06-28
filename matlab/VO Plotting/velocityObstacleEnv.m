classdef velocityObstacleEnv
    %Definition: This class creates an eviroment that can produce figures
    %that shows different objects in velocity space. 
    properties(Access = public)
        xVO;   %Set of velocity obsticle x-coornates
        yVO;   %Set of velocity obsticle y-coornates
        originVO;
        N    {mustBeNumeric} %Number of robots to simulate
        nFig {mustBeNumeric} %Number of figure to create
        tho  {mustBeNumeric} %Tho for velocity obstace
        R    {mustBeNumeric} %Radius of self + other robot
        listVO; %list of all VO objects robot numbers
    end
    properties(Access = private)
        lastTheta0VO = 0; %Used to deterine last angle for edge case
        f; %Figure object array for creating figures
        l; %Line object array to plot velocity obstacles 
        p; %Patch object array for filling in velocity Obs
        t; %Text object for numbering velocity obstaces
        a; %Axis object
        lVec; %Line to plot current robot's velocity
        halfPlane; %line object that holds half planes
        halfPlanePatch; %patch to shade in the half plane
        xMax;
        yMax;
    end
    
    
        methods
            function obj = velocityObstacleEnv(n)
                %Constructor
                obj.N = n; %Number of robots
            end
        end
        
        
        methods (Access = public)
            function obj = setRT(obj,r,t)
               %Setter for Tho and R 
               obj.R = r;
               obj.tho = t;
            end
            
            function obj = setVO(obj,pose,rNum)
                %This method output the velocity obstacles contained within
                %robot rNum's velocity space.
                %Pose should be a 2 X obj.N matrix holding positions
                %Vel should be a 2 X obj.N matrix holding velocities
                %Vel and Pose columns NEED TO BE IN ORDER by robot number
                
                obj.xVO = cell(obj.N-1,1);
                obj.yVO = cell(obj.N-1,1);
                for i = 1:(obj.N-1)
                    %this if statement is used to interate through the
                    %pose and Vel vector and get values from robots below
                    %rNum
                    if  rNum + i <= obj.N
                         %Finds the relative position/tho for the origin
                         %of the velocity obstacle circle and 
                         %other robot's current velocity.
                         pRelVec = (pose(:,rNum) - pose(:,rNum + i))/obj.tho;
                    else
                        %for robots below rNum 
                        pRelVec = (pose(:,rNum) - pose(:,rNum+i-obj.N))./obj.tho;
                    end 
                        obj.originVO(i,:)= -[pRelVec(1) pRelVec(2)] ;
                    %This condition determines if a vehicle collides
                    if (obj.R/obj.tho)/norm(pRelVec) < 1
                        thetaDiffVO = asin((obj.R/obj.tho)/norm(pRelVec));
                        obj.lastTheta0VO = thetaDiffVO;
                        theta0VO = atan2(pRelVec(2),pRelVec(1)) + pi; 
                        
                        VOint = sqrt(obj.R.^2 +norm(pRelVec).^2);
                        V = VOint:5:150;
                        
                        % theta bounds for VO
                        thetaVO = [(theta0VO - thetaDiffVO)*ones(1,length(V)); ...
                                    (theta0VO + thetaDiffVO)*ones(1,length(V))];
                
                        %Finds the theta diffrence for the arc
                        thetaArcDiff = (pi/2 - thetaDiffVO);
                        if pRelVec(2) < 0
                            thetaArc0 =  3*pi - thetaArcDiff + ...
                            acos(dot(pRelVec,[-1; 0])/norm(pRelVec));
                        else
                            thetaArc0 =  3*pi - thetaArcDiff - ...
                             acos(dot(pRelVec,[-1; 0])/norm(pRelVec));
                        end
                        
                        %Finds arc
                        [xVarc,yVarc] = arcCart(thetaArc0, ...
                                        thetaArc0 + 2*thetaArcDiff, ...
                                        obj.R/obj.tho,.1);
                        xVarc = xVarc - pRelVec(1);
                        yVarc = yVarc - pRelVec(2);
                        
                        
                        [xVO1, yVO1] = pol2cart(thetaVO(1,:),V);
                        [xVO2, yVO2] = pol2cart(thetaVO(2,:),V);
                        
                        %Creates final velocity obstacle
                            
                       
                            obj.xVO(i) = {[flip(xVO2) xVarc xVO1]};
                            obj.yVO(i) = {[flip(yVO2) yVarc yVO1]};

                    else
                        %Sets all figure object to the velocity obstacle
                        %when vehicles hit.
                        V =  0:5:200;
                        thetaVO = [(obj.lastTheta0VO - pi/2)*ones(1,length(V)); ...
                                    (obj.lastTheta0VO + pi/2)*ones(1,length(V))];
                        [xVO1, yVO1] = pol2cart(thetaVO(1,:),V);
                        [xVO2, yVO2] = pol2cart(thetaVO(2,:),V);
                        obj.xVO = {[xVO2 xVO1]};
                        obj.yVO = {[yVO2 yVO1]};
                    end
                end
            end
            
            function displayRelativeVO(obj,rNum,rNum2)
                %displays relative velocity for the robot rNum and rNum2 on
                %rNums figure
                        xVOtemp = cell2mat(obj.xVO(rNum2 - 1));
                        yVOtemp = cell2mat(obj.yVO(rNum2 - 1));
                        set(obj.l(rNum,rNum2 - 1),'xdata', xVOtemp, ...
                                         'ydata', yVOtemp);
                        set(obj.p(rNum,rNum2 - 1),'xdata',xVOtemp, ...
                                          'ydata',yVOtemp, ...
                                          'FaceColor', [0 1 1]);
                        
                        set(obj.t(rNum,rNum2 - 1),'position',[obj.originVO(rNum2 - 1,1), ...
                                                      obj.originVO(rNum2 - 1,2) 0], ...
                                                      'String', rNum + rNum2 - 1);
                        set(obj.f(rNum),'Name', ...
                        append('Relative Velocity Space For Robot ',string(rNum)));
            end
            
            function displayAgentVO(obj,rNum,robotsVel) 
                %Displays the figure of robot rNUM given the velocities of
                %all other agents robotsVel
                list = obj.listVO(1,:); 
               for i = 1:length(list)
                   xVOtemp = cell2mat(obj.xVO(list(i) - 1));
                   yVOtemp = cell2mat(obj.yVO(list(i) - 1));
                   if ~isempty(xVOtemp)
                        iRobotVelX = robotsVel(1,list(i))*ones(1,length(xVOtemp));
                        iRobotVelY = robotsVel(2,list(i))*ones(1,length(xVOtemp));
                   else
                       iRobotVelX = 0;
                       iRobotVelX = 0;
                       xVOtemp = 0;
                       yVOtemp = 0;
                   end
                   xVOtemp = xVOtemp + iRobotVelX;
                   yVOtemp = yVOtemp + iRobotVelY;
                   set(obj.l(rNum,list(i) - 1),'xdata', xVOtemp, ...
                                               'ydata', yVOtemp);
                   set(obj.p(rNum,list(i) - 1),'xdata',xVOtemp, ...
                                               'ydata',yVOtemp, ...
                                               'FaceColor', [0 1 1]);
                   set(obj.t(rNum,list(i) - 1),'position',[obj.originVO(list(i) - 1,1) + iRobotVelX(1), ...
                                                           obj.originVO(list(i) - 1,2) + iRobotVelY(1), 0], ...
                                                           'String', rNum + list(i) - 1);
               end
               set(obj.f(rNum),'Name', ...
                        append('Real Velocity Space For Robot ',string(rNum)));
            end
            
            
            
            function drawVector(obj,Velocity, rNum, lineNum)
                %Draws vector for a given velocity onto a figure of rNum
                set(obj.lVec(rNum, lineNum),'xdata',[Velocity(1) Velocity(2)], ...
                                   'ydata',[Velocity(3) Velocity(4)])
            end
            
            function obj = setPlot(obj,fNum,xMax,yMax)
                %Sets up given figures from x = [-xMax,xMax], y = [-yMax,yMax]
                    obj.f(fNum) = figure('Name',append('Velocity Space For Robot ',string(fNum))); 
                    obj.a(fNum) = axes;
                    ylabel('Vy')
                    xlabel('Vx')
                    xlim([-xMax,xMax])
                    ylim([-yMax,yMax])
                    obj.xMax = xMax;
                    obj.yMax = yMax;
            end
            
            function obj = addGraphicsVO(obj, fNUM, VOnum)
                %Adds a VO objects to the figure that allows the VO to be
                %displayed on the desired figure (fNUM). VOnum is used to
                %represent the desired robot VO to be saved.
                        obj.l(fNUM,VOnum - 1) = line('Parent', obj.a(fNUM));
                        obj.p(fNUM,VOnum - 1) = patch('Parent', obj.a(fNUM));
                        obj.t(fNUM,VOnum - 1) = text('Parent', obj.a(fNUM), ...
                                                     'Clipping', true);     
                        obj.listVO(fNUM,length(obj.listVO)+1) = VOnum;
            end
            
            function obj = addVector(obj, fNUM,color ,lineNumber) 
                % Adds a vector to the desired figure (fNUM) and is indexed
                % by lineNumber
                obj.lVec(fNUM, lineNumber) = line('Parent', obj.a(fNUM), ...
                                                  'color', color);
            end
            
            function obj = addHalfPlane(obj, fNUM, lineNumber)
                % Adds a halfplane to the desired figure (fNUM) and is indexed
                % by lineNumber
                  obj.halfPlane(fNUM, lineNumber) = line('Parent', obj.a(fNUM), ...
                                                         'visible', 'off');
                  obj.halfPlanePatch(fNUM, lineNumber) = patch('Parent', obj.a(fNUM), ...
                                                               'FaceColor', 'b', ...
                                                               'EdgeColor', 'none', ...
                                                               'visible', 'off');
            end
            
            
            function obj = drawHalfPlaneSpace(obj, fNum, psi, b, normalVector)
                newPsi = cell2mat(psi(fNum));
                if newPsi ~= 0
                    newB = cell2mat(b(fNum));
                    newNV = -cell2mat(normalVector(fNum));
                    for i = 1:(length(obj.halfPlane(fNum,:))-1)
                        if length(newPsi) >= i  
                            Ymax = tan(newPsi(i))*(obj.xMax + 10) + newB(i);
                            Ymin = tan(newPsi(i))*-(obj.xMax + 10) + newB(i);
%                             set(obj.halfPlane(fNum,i + fNum),'xdata', [-obj.xMax, obj.xMax], ...
%                                                        'ydata', [Ymin Ymax], ...
%                                                        'visible', 'on');  
                            YminP = 20*newNV(i,2) + Ymin;
                            YmaxP = 20*newNV(i,2) + Ymax;
                            XminP = 20*newNV(i,1) - (obj.xMax+10);
                            XmaxP = 20*newNV(i,1) + (obj.xMax+10) ;
                            set(obj.halfPlanePatch(fNum,i + fNum), 'xdata',[-(obj.xMax+10), (obj.xMax+10 ), XmaxP, XminP ]  ...
                                                           , 'ydata',[Ymin , Ymax , YmaxP , YminP ]...
                                                           , 'visible', 'on');
                        else
                            set(obj.halfPlane(fNum,i + fNum), 'visible', 'off');
                            set(obj.halfPlanePatch(fNum,i + fNum), 'visible', 'off'); 
                        end
                    end
                end 
            end
            
       end 
end
    

    