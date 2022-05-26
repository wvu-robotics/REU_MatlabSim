 
%% static boids rules optimizer

% used to optimize the boids rule set with static gains using bayesian
% optmization. to use the optimized values you must manually specify them
% in the "create_robot_swarm" function

% variables to optimize, name, range,      type
Ka = optimizableVariable('Ka', [0,10], 'Type', 'real'); % alignment gain
Ks = optimizableVariable('Ks', [0,10], 'Type', 'real'); % seperation gain
Kc = optimizableVariable('Kc', [0,10], 'Type', 'real'); % cohesion gain
Kh = optimizableVariable('Kh', [0,10], 'Type', 'real'); % home gain
Kg = optimizableVariable('Kg', [0,10], 'Type', 'real'); % goal gain

% funciton that serves as the simulation
% currently is optimizing for 10 agents with covariace intersection for 500
% time steps
fun = @(gains)optimizer(gains);

% allows you to run multiple trials of the optimization to see if any
% variables are not needed / to make sure you have reached the optimum
RESULTS = [];

for r = 1:1 % number of trials to run
    % display current itteration
    r
    % use bayesian optiization with the given simulation for a max of 100
    % itterations
    results = bayesopt(fun,[Ka,Ks,Kc,Kh,Kg],'MaxObjectiveEvaluations',100)
    RESULTS = [RESULTS,results];
    % overwrite results with all the completed trials that way if it
    % crashes then you still have all your data
    save('optimizer_results_CI_25', 'RESULTS'); 
    
end

    



% simulation function that passes in the boids gains and outputs the cost
% the cost follows this equation:
% cost = ((avg_covar)/cov_max 
%        +(avg_mean_error)/e_max 
%        +(avg_path_deviation)/total_goal_dist)
%       /((avg_goals_reached)/6.75);
% where 6.75 was the average goals reached with goal following only

function cost = optimizer(gains)

%% experiment parameters
num_agents = 25; % number of robots used in the simulator
estimator = 0;   % estimator 0 = dead reckoning
                 % estimator 1 = covariance intersection
                 % estimator 2 = decentralized EKF
                 % estimator 3 = centralized EKF
boids_rules = 1; % keep at 1 so it uses the static rules
enviorment = 0;  % enviorment 0 = random goal location
                 % enviorment 1 = foraging enviorment (not done)
headless = 1;    % headless 0 = display the simulation graphics
                 % headless 1 = do not display graphics

%% Simulation Parameters
simu.N=num_agents;          % number of Robots
range = 1;                  % robot detection range
e_max = 2;                  % maximum mean localization error
cov_max = 5;                % maximum covariance norm
simu.estimator = estimator; % estimator used for the experiment
simu.dt = .5;               % time step size
simu.simulationTime=500;    % total time steps duration (second)
simu.accumulatedTime=0;     % current simulator time
simu.i = 0;                 % current simulator time step

%noise models-------------------------------------------------------------
simu.percentNoiseDifference =0.01; %slightly varies sigma per agent
simu.sigmaVelocity=0.02;           %standard deviation of velocity errors (m/s)
simu.sigmaYawRate=0.01;            %standard deviation of yaw rate errors (rad/s)
simu.sigmaRange=.01;               %standard deviation of ranging measurement error (m)
simu.sigmaHeading =1*pi/180;       %standard deviation of heading measurment error (rad)
simu.biasVelocity=0.1*simu.sigmaVelocity; %standard deviation of velocity turn on bias (m/s)
simu.biasYawRate=0.1*simu.sigmaYawRate;   %standard deviation of yaw rate turn on bias (rad/s)
%---------------------------------------------------------------------------

simu.initdistance = range;   %distance between robots initial positions

%% Results Parameters

%keeps track of all results parameters by summing all each robot's value
%together for every timestep
%note: after the simulation ends the average value would be 
%       avg_value = value / (total_simultion_time * number_of_agents)

total_covar = 0;            % covariance over every time step
total_mean_error = 0;       % error between the dead reckoning and estimated position
total_goal_dist = 0;        % total straightline goal path
total_dist_traveled = 0;    % total distance covered by the robt
total_goals_reached = 0;    % number of goals the agents have reached

%% initialize swarm

show_detection_rng = 1;          %toggles on and off the detection range circles for graphics
rho_max = simu.N / (pi*range^2); % maximum robot density
world_len = 10;                  % specifies the world side length   

%create an array of Robot objects
ROBOTS = create_robot_swarm(simu.N,simu.initdistance,range); 

v_max = ROBOTS(1).max_speed; % get the robot's max speed

%dynamic percent area coverage -- needs further study to see how valueable
%this number is -- I believe it can be used for determining the
%connectivity of the swam
Crhro = simu.N*range*v_max*simu.simulationTime/(world_len^2)  

%give robots the noise models
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
    ROBOTS(r).home = [0,0];  % the home is at XY [0,0]
    ROBOTS(r).goal = [4*rand(1,1)+1, 4*rand(1,1)+1]; %give them a random goal
    ROBOTS(r).found_goal = 0; %specify that they have not found their goal
    ROBOTS(r).estimator = simu.estimator; %give the robot their esimator
    
    %givethe robots their gains
    ROBOTS(r).Kg = gains.Kg; 
    ROBOTS(r).Ka = gains.Ka;
    ROBOTS(r).Ks = gains.Ks;
    ROBOTS(r).Kc = gains.Kc;
    ROBOTS(r).Kh = gains.Kh;
    ROBOTS(r).Kv = 1;       % keep the velocity gain at 1 (not doing landmarks)
    
end

if headless == 0
    figure()  %object to display the simulator
end
tic

%% Simulation
while simu.accumulatedTime < simu.simulationTime
    %% measure the enviorment-----------------------
    
    %get every robot's CURRENT estimated position and estimated covariance
    [X,P] = ROBOTS(1).get_states(ROBOTS);
    for r = 1:simu.N
        % fill in all other robots' estimated position and covariance
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
        [states, covars] = ROBOTS(r).get_locations(ROBOTS); % uses X, P, and Lidar
        ROBOTS(r).state_particles{1} = states;
        ROBOTS(r).state_particles{2} = covars;
        
        % become a beacon following conditions in beacon update
        if boids_rules == 3 || boids_rules == 2
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
        ROBOTS(r) = ROBOTS(r).flock(ROBOTS(ROBOTS(r).neighbors));
        
        % get new goal if current one is found
        if ROBOTS(r).found_goal == 1
            if enviorment == 0 %generate a random goal if in that envioment setup
                ROBOTS(r).goal = [10*rand(1,1)-5, 10*rand(1,1)-5];
                ROBOTS(r).found_goal = 0;
            end
            %update the goal results parameters
            total_goal_dist = total_goal_dist + norm(ROBOTS(r).position_t(1,2) - ROBOTS(r).goal)-range;
            total_goals_reached = total_goals_reached + 1;
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
        pause(.003); % wait so the figure does not get instantly cleared
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

% final cost function
cost = ((avg_covar)/cov_max + (avg_mean_error)/e_max + (avg_path_deviation)/total_goal_dist)/((avg_goals_reached)/6.75)

disp("===================================================================")

end



