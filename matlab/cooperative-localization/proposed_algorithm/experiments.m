function [cost, avg_mean_error, avg_covar, avg_path_deviation, avg_goals_reached] = experiments(estimator,boids_rules,enviorment,headless,num_agents)
%% EXPERIMENTS runs the simulation while changing parameters for simulation results
%   ====================================================================
%   estimator :     [int]   0 = dead reckoning
%                           1 = covariance intersection
%                           2 = decentralized EKF
%                           3 = Centralized EKF
%
%   boids_rules :   [int]   0 = Goal only
%                           1 = Static rules
%                           2 = Adaptive rules
%                           3 = Adaptive rules with beacons ???
%
%   enviorment :    [int]   0 = random waypoint generator
%                           1 = foraging senario
%
%   headless :      [int]   0 = use display functions
%                           1 = do not display 
%
%   ===================================================================
%
%   cost = (avg_mean_error + avg_covar + avg_path_deviation)/avg_goals_reached

%% adaptive boids rule simulator

%% Simulation Parameters
simu.N=num_agents;           % number of Robots
range = 1;           % robot detection range [m]
e_max = 2;           % maximum mean localization error [m]
cov_max = 5;         % maximum covariance norm [m^2]
simu.estimator = estimator;  
simu.dt = .5; % time step size [sec]
simu.simulationTime=100;   %flight duration [sec]
simu.accumulatedTime=0;  %first timestep
simu.initdistance = range;   %distance between each pair of neighbot UAVs' initial positions
%noise models-------------------------------------------------------------
simu.percentNoiseDifference =0.01; %slightly varies sigma per agent
simu.sigmaVelocity=0.05;  %standard deviation of velocity errors (m/s)
simu.sigmaYawRate=0.01;    %standard deviation of yaw rate errors (rad/s)
simu.sigmaRange=.01;   %standard deviation of ranging measurement error (m)
simu.sigmaHeading =1*pi/180; %standard deviation of heading measurment error from laser (rad)
simu.biasVelocity=0.1*simu.sigmaVelocity; %standard deviation of velocity turn on bias (m/s)
simu.biasYawRate=0.1*simu.sigmaYawRate; %standard deviation of yaw rate turn on bias (rad/s)
%---------------------------------------------------------------------------



%% Results Parameters

%keeps track of all results parameters by summing all each robot's value
%together for every timestep
%note: after the simulation ends the average value would be 
%       avg_value = value / (total_simultion_time * number_of_agents)

total_covar = 0;            % covariance over every time step
total_mean_error = 0;       % error between the dead reckoning and estimated position
total_goal_dist = 0;        % total straightline goal path
total_dist_traveled = 0;    % total distance covered by the robt
total_goals_reached = 0;    % number of goals the agents have actually reached
total_false_goals_reached = 0; % number of goals the agents though they reached but did not

 %% initialize swarm
 
 show_detection_rng = 1;  %toggles on and off the detection range circles
 rho_max = simu.N / (pi*range^2); % maximum robot density
 world_len = 10;                  % side length of the world [m]
 
 ROBOTS = create_robot_swarm(simu.N,simu.initdistance,range); %create an array of Boids models for the robots
 v_max = ROBOTS(1).max_speed;
 
 %dynamic percent area coverage -- needs further study to see how valueable
%this number is -- I believe it can be used for determining the
%connectivity of the swam
 Crhro = simu.N*range*v_max*simu.simulationTime/(world_len^2)  %dynamic percent area coverage
 
 %give robot's noise models
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
    ROBOTS(r).home = [0,0]; % the home is at XY [0,0]
    ROBOTS(r).goal = [4*rand(1,1)+1, 4*rand(1,1)+1]; %give them a random goal
    ROBOTS(r).found_goal = 0; %specify that they have not found their goal
    ROBOTS(r).estimator = simu.estimator;  %give the robot their esimator
    
    % if boids_rules = goal only set a high gain for goals and nothing else
    if boids_rules == 0
        ROBOTS(r).Kg = 1000;
        ROBOTS(r).Ka = 0;
        ROBOTS(r).Ks = 0;
        ROBOTS(r).Kc = 0;
        ROBOTS(r).Kh = 0;
        ROBOTS(r).Kv = 1;  % keep the velocity gain at 1 (not doing landmarks)
    end
end

if headless == 0
    figure()  %object to display the simulator
end
 tic
 
 %% Simulation
disp('Performing simulation...');
simu.i=2;
while simu.accumulatedTime < simu.simulationTime  
    %% measure the enviorment-----------------------
   
   %get every robot's CURRENT estimated position and estimated covariance
   
    
   for r = 1:simu.N
        % all other robots' estimated position and covariance
       if simu.i == 2 || estimator ~= 2 || estimator ~= 3 
            [X,P] = ROBOTS(r).get_states(ROBOTS);
       end
        ROBOTS(r).X = X;
        ROBOTS(r).P = P;
       
        % range and bearing to all other robots from  CURRENT Truth Position
        ROBOTS(r) = ROBOTS(r).lidar_measurement(ROBOTS);
        % velocity magnitude and yaw rate from CURRENT True Velocity 
        %(carries CURRENT_POSITION -> NEXT_POSITION)
        ROBOTS(r) = ROBOTS(r).encoder_measurement();
         
        
   end
   
   %% perception------------------------------------
   for r = 1:simu.N 
        % get prediction of my location from the other robots
        % state = [X;Y,Yaw] and covar = [3x3]
        [states, covars] = ROBOTS(r).get_locations(ROBOTS); % uses X, P, and Lidar (CURRENT)
        ROBOTS(r).state_particles{1} = states;
        ROBOTS(r).state_particles{2} = covars;
        
        % become a beacon following conditions in beacon update
        if boids_rules == 3
            ROBOTS = ROBOTS(r).beacon_update(ROBOTS, cov_max);
        end
   end
    
    %% take action and update------------------------
    for r = 1:simu.N
          
        % enviorment act on robot
        ROBOTS(r) = ROBOTS(r).update(); % update CURRENT -> NEXT positon
        
        % adapt the boids rules based on local stimuli and allowable
        % thresholds
        if boids_rules == 2 || boids_rules == 3
            ROBOTS(r) = ROBOTS(r).boids_update(e_max,rho_max,cov_max,world_len);
        end
        
        % based on boids rules set the next velocity
        ROBOTS(r) = ROBOTS(r).flock(ROBOTS(ROBOTS(r).neighbors)); % CURRENT -> NEXT velocity
        
        % get new goal if current one is found
        if ROBOTS(r).found_goal == 1
            if enviorment == 0 %generate a random goal if in that envioment setup
                ROBOTS(r).goal = [10*rand(1,1)-5, 10*rand(1,1)-5]; %[ROBOTS(r).goal(1),-ROBOTS(r).goal(2)];
                ROBOTS(r).found_goal = 0;
            end
            %update the goal results parameters
            total_goal_dist = total_goal_dist + norm(ROBOTS(r).position_t(1,2) - ROBOTS(r).goal)-range;
            
            %check to see if we actually reached the goal
            if norm(ROBOTS(r).position_t(1:2) - ROBOTS(r).goal) < ROBOTS(r).detection_range
                total_goals_reached = total_goals_reached + 1;
            else
                total_false_goals_reached = total_false_goals_reached+1;
            end
        end
        
        %update results parameters
        total_covar = total_covar + norm(ROBOTS(r).covariance_e);
        total_mean_error = total_mean_error + norm(ROBOTS(r).position_e(1:2) - ROBOTS(r).position_t(1:2));
        total_dist_traveled = total_dist_traveled + norm(ROBOTS(r).path_t(end-1,1:2) - ROBOTS(r).position_t(1:2));
        
    end
    
    %% display and increase time-------------------------
     
    %display the current state of the swarm
     
     if headless == 0
        disp_swarm(ROBOTS,range,show_detection_rng); %graph simulation
        disp(simu.accumulatedTime); %output the current simulation time
        pause(.003);% wait so the figure does not get instantly cleared
     end
    %Update Simulation Variables
    simu.i = simu.i + 1;
    simu.accumulatedTime = simu.accumulatedTime + simu.dt;
end

%% results

% find the average results across all time steps per robot
avg_covar = total_covar/(simu.N*simu.i)
avg_mean_error = total_mean_error/(simu.N*simu.i)
avg_path_deviation = (total_dist_traveled-total_goal_dist)/(simu.N*simu.i)
avg_goals_reached = total_goals_reached/simu.N

% TODO add in the false goals reached if needed

% final cost function
cost = ((avg_covar)/cov_max + (avg_mean_error)/e_max + (avg_path_deviation)/total_goal_dist)/((avg_goals_reached)/6.75);

disp("===================================================================")
 
end

