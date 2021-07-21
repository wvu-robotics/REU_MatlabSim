% cooperative localization main function



%% adaptive boids rule simulator

clear all; 
close all; 
clc;

%% Simulation Parameters
simu.N=10;           % number of Robots
range = 5;           % robot detection range
e_max = 2;           % maximum mean localization error
cov_max = 5;         % maximum covariance norm
simu.estimator = 1;  % 0 = dead reckoning
                     % 1 = covariance intersection
                     % 2 = decentralized EKF
                     % 3 = centralized EKF;
simu.dt = .5; % time step size
simu.simulationTime=100;   %flight duration (second)
simu.accumulatedTime=0;  %first timestep

simu.percentNoiseDifference =0.01; %slightly varies sigma per agent
simu.sigmaVelocity=0.02;  %standard deviation of velocity errors (m/s)
simu.sigmaYawRate=0.01*pi/180;    %standard deviation of yaw rate errors (rad/s)
simu.sigmaRange=.001;   %standard deviation of ranging measurement error (m)
simu.sigmaHeading =1*pi/180; %standard deviation of heading measurment error (rad)
simu.biasVelocity=0.1*simu.sigmaVelocity; %standard deviation of velocity turn on bias (m/s)
simu.biasYawRate=0.1*simu.sigmaYawRate; %standard deviation of yaw rate turn on bias (rad/s)

simu.initdistance = 5;   %distance between each pair of neighbot UAVs' initial positions

%% Results Parameters

total_covar = 0;
total_mean_error = 0;
total_goal_dist = 0;
total_dist_traveled = 0;
total_goals_reached = 0;

 %% initialize swarm
 
 show_detection_rng = 1;  %toggles on and off the detection range circles
 v_max = 5;         % maximum velocity
 rho_max = simu.N / (pi*range^2); % maximum robot density
 world_len = 100;
 Crhro = simu.N*range*v_max*simu.simulationTime/(world_len^2)  %dynamic percent area coverage

 ROBOTS = create_robot_swarm(simu.N,simu.initdistance,range); %create an array of Boids models for the robots
 
 for idx=1:simu.N
    %noise
    ROBOTS(idx).sigmaVelocity = simu.sigmaVelocity - (simu.percentNoiseDifference)*simu.sigmaVelocity + 2*rand*(simu.percentNoiseDifference)*simu.sigmaVelocity;
    ROBOTS(idx).sigmaYawRate = simu.sigmaYawRate - (simu.percentNoiseDifference)*simu.sigmaYawRate + 2*rand*(simu.percentNoiseDifference)*simu.sigmaYawRate;
    ROBOTS(idx).biasVelocity = normrnd(0,simu.biasVelocity);
    ROBOTS(idx).biasYawRate = normrnd(0,simu.biasYawRate);
    ROBOTS(idx).sigmaRange = simu.sigmaRange;
    ROBOTS(idx).sigmaHeading = simu.sigmaHeading;
    ROBOTS(idx).dt = simu.dt;
 end
 
 %give the robots home and goal location and esimator
for r = 1:simu.N
    ROBOTS(r).home = [0,0];
    ROBOTS(r).goal = [10*rand(1,1)+35, 10*rand(1,1)-35];
    ROBOTS(r).Kg = 1000;
    ROBOTS(r).found_goal = 0;
    ROBOTS(r).estimator = simu.estimator; 
end

figure()  %object to display the simulator

 
 tic
 
 %% Simulation
disp('Performing simulation...');
simu.i=2;
while simu.accumulatedTime < simu.simulationTime  
   %% measure the enviorment-----------------------
   
   %get every robot's CURRENT estimated position and estimated covariance
   [X,P] = ROBOTS(1).get_states(ROBOTS); 
   for r = 1:simu.N
        % all other robots' estimated position and covariance
        ROBOTS(r).X = X;
        ROBOTS(r).P = P;
       
        % range and bearing from  CURRENT Truth Position 
        ROBOTS(r) = ROBOTS(r).lidar_measurement(ROBOTS);
        % velocity magnitude and yaw rate from CURRENT True Velocity 
        %(carries CURRENT_POSITION -> NEXT_POSITION)
        ROBOTS(r) = ROBOTS(r).encoder_measurement(); 
        
   end
   
   %% perception------------------------------------
   for r = 1:simu.N 
        % get prediction of my location from the other robots
        [states, covars] = ROBOTS(r).get_locations(ROBOTS); % uses X, P, and Lidar (CURRENT)
        ROBOTS(r).state_particles{1} = states;
        ROBOTS(r).state_particles{2} = covars;
        
        % become a beacon
      %  ROBOTS = ROBOTS(r).beacon_update(ROBOTS, cov_max);
   end
    
    %% take action and update------------------------
    for r = 1:simu.N
          
        % enviorment act on robot
        ROBOTS(r) = ROBOTS(r).update(); % update CURRENT -> NEXT positon
        
        % robot's velocity controller
       % ROBOTS(r) = ROBOTS(r).boids_update(e_max,rho_max,cov_max,world_len);
        ROBOTS(r) = ROBOTS(r).flock(ROBOTS(ROBOTS(r).neighbors)); % CURRENT -> NEXT velocity
        
        % get new goal if current one is found
        if ROBOTS(r).found_goal == 1
            ROBOTS(r).goal = [100*rand(1,1)-50, 100*rand(1,1)-50]; %[ROBOTS(r).goal(1),-ROBOTS(r).goal(2)];%
            ROBOTS(r).Kg = 1000;
            ROBOTS(r).found_goal = 0;
            
            total_goal_dist = total_goal_dist + norm(ROBOTS(r).position_t(1,2) - ROBOTS(r).goal);
            total_goals_reached = total_goals_reached + 1;
        end
        
        total_covar = total_covar + norm(ROBOTS(r).covariance_e);
        total_mean_error = total_mean_error + norm(ROBOTS(r).position_e(1:2) - ROBOTS(r).position_t(1:2));
        
        total_dist_traveled = total_dist_traveled + norm(ROBOTS(r).path_t(end-1,1:2) - ROBOTS(r).position_t(1:2));
        
    end
    
    %% display and increase time-------------------------
     
    %display the current state of the swarm
     disp(simu.accumulatedTime);
     disp_swarm(ROBOTS,range,show_detection_rng);
     pause(.003);
    %Update Simulation Variables
    simu.i = simu.i + 1;
    simu.accumulatedTime = simu.accumulatedTime + simu.dt;
end

avg_covar = total_covar/(simu.N*simu.i)
avg_mean_error = total_mean_error/(simu.N*simu.i)
avg_path_deviation = (total_goal_dist - total_dist_traveled)/(simu.N*simu.i)
avg_goals_reached = total_goals_reached/simu.N 
 
 
 