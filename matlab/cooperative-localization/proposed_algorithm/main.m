% cooperative localization main function used to test single case senarios
% robot is based on SMART2 robots

%% Parameter Key ==========================================================
%     estimator :     [int]   0 = dead reckoning
%                             1 = covariance intersection
%                             2 = decentralized EKF
%                             3 = Centralized EKF
%
%     boids_rules :   [int]   0 = Goal only
%                             1 = Static rules
%                             2 = Adaptive rules
%                             3 = Adaptive rules with beacons (TODO)
%
%     enviorment :    [int]   0 = random waypoint generator
%                             1 = foraging senario (TODO)
%
%     headless :      [int]   0 = use display functions
%                             1 = do not display
%
%     num_agents :    [int]   5 = low number of agents
%                            10 = moderate number of agents
%                            25 = high number of agents
%                            50 = very high number of agents
             
%% simulation parameters

num_agents = 10;   % number of robots used
enviorment = 0;    % enviroment setting
boids_rules = 2;   % boids rule set
estimator = 0;     % localization estimator
headless = 0;      % displays the simulation

%% run simulation
[cost, avg_mean_error, avg_covar, avg_path_deviation, avg_goals_reached] = experiments(estimator,boids_rules,enviorment,headless,num_agents)














