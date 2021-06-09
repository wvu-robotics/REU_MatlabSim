clc
clear
% initiating Vehciles 
Vmax = 10;
n = 4;
env = MultiRobotEnv(n);
env.robotRadius = 1;
env.hasWaypoints = true;
env.showTrajectory = false;
L = .3;
R = 1;
vehicle =  DifferentialDrive(R,L);

%simulation param
 sampleTime = .1;
 t = 0:.1:15;
 tho = 1;
 p = zeros(3,length(t),n);
 goal = zeros(n,2);
 for i = 1:n
     p(:,1,i) = [randi([-5,5]) randi([-5,5]) 2*pi*rand(1)];
     vel(:,1,i) = [0 0 0];
     goal(i,:) = [randi([-10,10]), randi([-10,10])];
 end
 


controller = controllerPurePursuit;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = Vmax;
controller.MaxAngularVelocity = 1.5;




close all
r = rateControl(1/sampleTime);
R = 2*env.robotRadius;

VOenv = velocityObstacleEnv(n,1);
VOenv = VOenv.setRT(R, tho);
VOenv = VOenv.setPlot(50,50);


for T = 2:length(t)
    for i = 1:n
        % Run the Pure Pursuit controller and convert output to wheel speeds
        controller.Waypoints = goal(i,:);
        [vRef,wRef] = controller(p(:,T-1,i));
        [wL,wR] = inverseKinematics(vehicle,vRef,wRef);
    
        % Compute the velocities
        [v,w] = forwardKinematics(vehicle,wL,wR);
        
        velB = [v;0;w]; % Body velocities [vx;vy;w]
        vel(:,T-1,i) = bodyToWorld(velB,p(:,T-1,i));  % Convert from body to world
        
        % Perform forward discrete integration step
        p(:,T,i) = p(:,T-1,i) + vel(:,T-1,i)*sampleTime; 

        for j = 1:n
            NewPose(1:2,j) = p(1:2,T-1,j);
            NewVel(1:2,j) = vel(1:2,T-1,j);
        end     
        %For mulitple plots
%             VOenv = VOenv.displayVO( NewPose,NewVel,i);
%             VOenv.drawVector(NewVel(:,i),i);
    end
    %For one plot.
    VOenv = VOenv.displayVO( NewPose,NewVel,1);
    VOenv.drawVector(NewVel(:,1),1);
    
    
    % Update visualization
    env(1:n,p(:,T-1,:),goal);
    waitfor(r);
end 