
%% static boids rules optimizer

Ka = optimizableVariable('Ka', [0,10], 'Type', 'real');
Ks = optimizableVariable('Ks', [0,10], 'Type', 'real');
Kc = optimizableVariable('Kc', [0,10], 'Type', 'real');
Kh = optimizableVariable('Kh', [0,10], 'Type', 'real');
Kg = optimizableVariable('Kg', [0,10], 'Type', 'real');

fun = @(gains)optimizer(gains);

RESULTS = [];

for r = 1:1
    r
    results = bayesopt(fun,[Ka,Ks,Kc,Kh,Kg],'MaxObjectiveEvaluations',100)
    RESULTS = [RESULTS,results];
    save('optimizer_results', 'RESULTS');
end




function cost = optimizer(gains)

num_agents = 10;
estimator = 1;
boids_rules = 1;
enviorment = 0;
headless = 1;

%% Simulation Parameters
simu.N=num_agents;           % number of Robots
range = 1;           % robot detection range
e_max = 2;           % maximum mean localization error
cov_max = 5;         % maximum covariance norm
simu.estimator = estimator;
simu.dt = .5; % time step size
simu.simulationTime=500;   %flight duration (second)
simu.accumulatedTime=0;  %first timestep

%noise models-------------------------------------------------------------
simu.percentNoiseDifference =0.01; %slightly varies sigma per agent
simu.sigmaVelocity=0.02;  %standard deviation of velocity errors (m/s)
simu.sigmaYawRate=0.01;    %standard deviation of yaw rate errors (rad/s)
simu.sigmaRange=.01;   %standard deviation of ranging measurement error (m)
simu.sigmaHeading =1*pi/180; %standard deviation of heading measurment error (rad)
simu.biasVelocity=0.1*simu.sigmaVelocity; %standard deviation of velocity turn on bias (m/s)
simu.biasYawRate=0.1*simu.sigmaYawRate; %standard deviation of yaw rate turn on bias (rad/s)
%---------------------------------------------------------------------------

simu.initdistance = range;   %distance between each pair of neighbot UAVs' initial positions

%% Results Parameters

total_covar = 0;
total_mean_error = 0;
total_goal_dist = 0;
total_dist_traveled = 0;
total_goals_reached = 0;

%% initialize swarm

show_detection_rng = 1;  %toggles on and off the detection range circles
rho_max = simu.N / (pi*range^2); % maximum robot density
world_len = 10;

ROBOTS = create_robot_swarm(simu.N,simu.initdistance,range); %create an array of Boids models for the robots
v_max = ROBOTS(1).max_speed;
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
    ROBOTS(r).home = [0,0];
    ROBOTS(r).goal = [4*rand(1,1)+1, 4*rand(1,1)+1];
    ROBOTS(r).found_goal = 0;
    ROBOTS(r).estimator = simu.estimator;
    
    ROBOTS(r).Kg = gains.Kg;
    ROBOTS(r).Ka = gains.Ka;
    ROBOTS(r).Ks = gains.Ks;
    ROBOTS(r).Kc = gains.Kc;
    ROBOTS(r).Kh = gains.Kh;
    ROBOTS(r).Kv = 1;
    
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
        if boids_rules == 3 || boids_rules == 2
            ROBOTS = ROBOTS(r).beacon_update(ROBOTS, cov_max);
        end
    end
    
    %% take action and update------------------------
    for r = 1:simu.N
        
        % enviorment act on robot
        ROBOTS(r) = ROBOTS(r).update(); % update CURRENT -> NEXT positon
        
        % robot's velocity controller
        if boids_rules == 2 || boids_rules == 3
            ROBOTS(r) = ROBOTS(r).boids_update(e_max,rho_max,cov_max,world_len);
        end
        ROBOTS(r) = ROBOTS(r).flock(ROBOTS(ROBOTS(r).neighbors)); % CURRENT -> NEXT velocity
        
        % get new goal if current one is found
        if ROBOTS(r).found_goal == 1
            if enviorment == 0
                ROBOTS(r).goal = [10*rand(1,1)-5, 10*rand(1,1)-5]; %[ROBOTS(r).goal(1),-ROBOTS(r).goal(2)];
                ROBOTS(r).found_goal = 0;
            end
            total_goal_dist = total_goal_dist + norm(ROBOTS(r).position_t(1,2) - ROBOTS(r).goal)-range;
            total_goals_reached = total_goals_reached + 1;
        end
        
        total_covar = total_covar + norm(ROBOTS(r).covariance_e);
        total_mean_error = total_mean_error + norm(ROBOTS(r).position_e(1:2) - ROBOTS(r).position_t(1:2));
        total_dist_traveled = total_dist_traveled + norm(ROBOTS(r).path_t(end-1,1:2) - ROBOTS(r).position_t(1:2));
        
    end
    
    %% display and increase time-------------------------
    
    %display the current state of the swarm
    
    if headless == 0
        disp_swarm(ROBOTS,range,show_detection_rng);
        disp(simu.accumulatedTime);
        pause(.003);
    end
    %Update Simulation Variables
    simu.i = simu.i + 1;
    simu.accumulatedTime = simu.accumulatedTime + simu.dt;
end

%% results
avg_covar = total_covar/(simu.N*simu.i)
avg_mean_error = total_mean_error/(simu.N*simu.i)
avg_path_deviation = (total_dist_traveled-total_goal_dist)/(simu.N*simu.i)
avg_goals_reached = total_goals_reached/simu.N

%cost = (avg_mean_error + avg_covar + avg_path_deviation)/avg_goals_reached
%cost = (avg_covar-cov_max)/cov_max + (avg_mean_error-e_max)/e_max + (avg_path_deviation)/total_goal_dist+(6.75-avg_goals_reached)/6.75
cost = ((avg_covar)/cov_max + (avg_mean_error)/e_max + (avg_path_deviation)/total_goal_dist)/((avg_goals_reached)/6.75);

disp("===================================================================")

end



