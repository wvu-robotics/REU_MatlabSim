classdef velocityObstacleEnv
    %Definition: This class creates an eviroment that can produce figures
    %that shows different objects in velocity space. 
    properties(Access = public)
        xVO  {mustBeNumeric} %Set of velocity obsticle x-coornates
        yVO  {mustBeNumeric} %Set of velocity obsticle y-coornates
        N    {mustBeNumeric} %Number of robots to simulate
        nFig {mustBeNumeric} %Number of figure to create
        tho  {mustBeNumeric} %Tho for velocity obstace
        R    {mustBeNumeric} %Radius of self + other robot
    end
    properties(Access = private)
        lastTheta0VO = 0; %Used to deterine last angle for edge case
        f; %Figure object array for creating figures
        l; %Line object array to plot velocity obstacles 
        p; %Patch object array for filling in velocity Obs
        t; %Text object for numbering velocity obstaces
        a; %Axis object
        lVec; %Line to plot current robot's velocity 
    end
    
    
        methods
            function obj = velocityObstacleEnv(n,nFig)
                %Constructor
                obj.N = n; %Number of robots
                obj.nFig = nFig; %Number of figures
            end
        end
        
        
        methods (Access = public)
            function obj = setRT(obj,r,t)
               %Setter for Tho and R 
               obj.R = r;
               obj.tho = t;
            end
            
            function obj = displayVO(obj,pose,vel, rNum)
                %This method output the velocity obstacles contained within
                %robot rNum's velocity space.
                %Pose should be a 2 X obj.N matrix holding positions
                %Vel should be a 2 X obj.N matrix holding velocities
                %Vel and Pose columns NEED TO BE IN ORDER by robot number
                
                for i = 1:(obj.N-1)
                    %this if statement is used to interate through the
                    %pose and Vel vector and get values from robots below
                    %rNum
                    if  rNum + i <= obj.N
                         %Finds the relative position/tho for the origin
                         %of the velocity obstacle circle and 
                         %other robot's current velocity.
                         pRelVec = (pose(:,rNum) - pose(:,rNum + i))/obj.tho;
                         Vb = vel(:,rNum + i); 
                         set(obj.t(rNum,i),'position',[-pRelVec(1) + Vb(1), ...
                                                       -pRelVec(2) + Vb(2) 0], ...
                                                        'String', rNum + i);
                    else
                        %for robots below rNum 
                        pRelVec = (pose(:,rNum) - pose(:,rNum+i-obj.N))./obj.tho;
                        Vb = vel(:,rNum+i-obj.N);
                        
                        set(obj.t(rNum,i),'position',[-pRelVec(1) + Vb(1), ...
                                                       -pRelVec(2) + Vb(2) 0], ...
                                                        'String',rNum + i-obj.N);
                    end 
                      
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
                        obj.xVO = [xVO2 xVarc xVO1];
                        obj.yVO = [yVO2 yVarc yVO1];
                        
                        
                        %Sets all figure object to the velocity obstacle
                        obj.xVO = obj.xVO + Vb(1)*ones(1,length(obj.xVO));
                        obj.yVO = obj.yVO + Vb(2)*ones(1,length(obj.yVO));
                        
                        xVOp = [ xVarc xVO1 flip(xVO2)];
                        yVOp = [yVarc yVO1 flip(yVO2)];
                        
                        xVOp = xVOp + Vb(1)*ones(1,length(obj.xVO));
                        yVOp = yVOp + Vb(2)*ones(1,length(obj.yVO));
                
                        set(obj.l(rNum,i),'xdata',real(obj.xVO), ...
                                          'ydata',real(obj.yVO));
                
                        set(obj.p(rNum,i),'xdata',xVOp, ...
                                          'ydata',yVOp, 'FaceColor', [0 1 1]);
                    else
                        %Sets all figure object to the velocity obstacle
                        %when vehicles hit.
                        V =  0:5:200;
                        thetaVO = [(obj.lastTheta0VO - pi/2)*ones(1,length(V)); ...
                                    (obj.lastTheta0VO + pi/2)*ones(1,length(V))];
                        [xVO1, yVO1] = pol2cart(thetaVO(1,:),V);
                        [xVO2, yVO2] = pol2cart(thetaVO(2,:),V);
                        obj.xVO = [xVO2 xVO1];
                        obj.yVO = [yVO2 yVO1];
                        
                        obj.xVO = obj.xVO + Vb(1)*ones(1,length(obj.xVO));
                        obj.yVO = obj.yVO + Vb(2)*ones(1,length(obj.yVO));
                        set(obj.l(rNum,i),'xdata',real(obj.xVO), ...
                                      'ydata',real(obj.yVO));
                        set(obj.p(rNum,i),'xdata',real(obj.xVO), ...
                                          'ydata',real(obj.yVO));
                    end
                end
            end
            
            function drawVector(obj,Velocity, rNum)
                %Draws vector for a given velocity onto a figure of rNum
                set(obj.lVec(rNum),'xdata',[0 Velocity(1)], ...
                                   'ydata',[0 Velocity(2)], ...
                                   'color', [1 0 0]); 
            end
            
            function obj = setPlot(obj,xMax,yMax)
                %Sets up given figures from x = [-xMax,xMax], y = [-yMax,yMax]
                for i = 1:obj.nFig
                    obj.f(i) = figure;
                    obj.a(i) = axes;
                    xlim([-xMax,xMax])
                    ylim([-yMax,yMax])
                    obj.lVec(i) = line;
                    for j = 1:(obj.N-1)
                        obj.l(i,j) = line;
                        obj.p(i,j) = patch;
                        obj.t(i,j) = text;
                    end
                end
            end
       end 
end
    

    