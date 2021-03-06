%% compiles all the data from generate data to plot the results



set(0, 'DefaultFigureColor', 'white');

%% results matricies
NAMES = [];
ALL_ERROR = [];
ALL_COVAR = [];
ALL_GOALS = [];
ALL_COST = [];
ALL_PATH = [];
ALL_FALSE_GOALS = [];


RULE_COLOR = [];
ESTIMATOR = [];
SIZE = [];

DR_GOAL = [];
DR_STATIC = [];
DR_ADAPT = [];

CI_GOAL = [];
CI_STATIC = [];
CI_ADAPT = [];

DEKF_GOAL = [];
DEKF_STATIC = [];
DEKF_ADAPT = [];

CEKF_GOAL = [];
CEKF_STATIC = [];
CEKF_ADAPT = [];

%% data selection
% each of these for loops specififes which data you want to load to see
i = 1;
estimator_methods = [0,1,2];%[0,1,2,3] 
num_esimators = size(estimator_methods);
for estimator = estimator_methods
    for enviorment = 0
        for boids_rules = [0,1,2]
            for num_agents = [5,10,25,50] %[5,10,25,50]
                           
                %% file name generator
                switch estimator
                    case 0
                        est = 'dr ';
                    case 1
                        est = 'ci ';
                    case 2
                        est = 'dekf ';
                    case 3
                        est = 'cekf ';
                    otherwise
                        est = 'error ';
                end
                
                switch boids_rules
                    case 0
                        rule = 'goal ';
                    case 1
                        rule = 'static ';
                    case 2
                        rule = 'adaptive ';
                    case 3
                        rule = 'beacon ';
                    otherwise
                        rule = 'error ';
                end
                
                switch enviorment
                    case 0
                        env = 'random';
                    case 1
                        env = 'foraging';
                    otherwise
                        env = 'error';
                end
                
                num = string(num_agents);
                %% load in the data
                file_name = "data/" + est + rule + env + " " + num + ".mat";
                load(file_name);
               
                ALL_ERROR = [ALL_ERROR,MEAN_ERROR'];
                ALL_COVAR = [ALL_COVAR,COVAR'];
                ALL_GOALS = [ALL_GOALS,GOALS_REACHED'];
                ALL_COST = [ALL_COST,COST'];
                ALL_PATH = [ALL_PATH,PATH_DEVIATION'];  
                ALL_FALSE_GOALS = [ALL_FALSE_GOALS,(100*GOALS_REACHED./(FALSE_GOALS_REACHED+GOALS_REACHED))'];
             
                if estimator == 0
                    if boids_rules == 0
                        DR_GOAL = [DR_GOAL,i];
                    elseif boids_rules == 1
                        DR_STATIC = [DR_STATIC,i];
                    elseif boids_rules == 2
                        DR_ADAPT = [DR_ADAPT,i];
                    end
                elseif estimator == 1
                    if boids_rules == 0
                        CI_GOAL = [CI_GOAL,i];
                    elseif boids_rules == 1
                        CI_STATIC = [CI_STATIC,i];
                    elseif boids_rules == 2
                        CI_ADAPT = [CI_ADAPT,i];
                    end
                elseif estimator == 2
                    if boids_rules == 0
                        DEKF_GOAL = [DEKF_GOAL,i];
                    elseif boids_rules == 1
                        DEKF_STATIC = [DEKF_STATIC,i];
                    elseif boids_rules == 2
                        DEKF_ADAPT = [DEKF_ADAPT,i];
                    end
                elseif estimator == 3
                    if boids_rules == 0
                        CEKF_GOAL = [CEKF_GOAL,i];
                    elseif boids_rules == 1
                        CEKF_STATIC = [CEKF_STATIC,i];
                    elseif boids_rules == 2
                        CEKF_ADAPT = [CEKF_ADAPT,i];
                    end
                end


                if boids_rules == 0
                    RULE_COLOR = [RULE_COLOR, [1;0;0]];
                elseif boids_rules == 1
                    RULE_COLOR = [RULE_COLOR, [0;1;0]];
                else
                   RULE_COLOR = [RULE_COLOR, [0;0;1]]; 
                end
                
                if estimator == 0
                    ESTIMATOR = [ESTIMATOR, 'x'];
                elseif estimator == 1
                    ESTIMATOR = [ESTIMATOR, 'o'];
                elseif estimator == 2
                   ESTIMATOR = [ESTIMATOR, 's']; 
                else
                   ESTIMATOR = [ESTIMATOR, '^'];
                end
                
                if num_agents == 5
                    SIZE = [SIZE,5];
                elseif num_agents == 10
                    SIZE = [SIZE,10];
                elseif num_agents == 25
                    SIZE = [SIZE,25];
                else
                    SIZE = [SIZE, 50];
                end

                 

               i = i+1;
                
            end
        end
    end
end

%% metrics

% remove outliers (3 Mean absolute deviations)
[~, error_out] = rmoutliers(ALL_ERROR, 'mean');
[~, covar_out] = rmoutliers(ALL_COVAR, 'mean');
[~, goals_out] = rmoutliers(ALL_GOALS, 'mean');
[~, path_out] = rmoutliers(ALL_PATH, 'mean');
[~, cost_out] = rmoutliers(ALL_COST, 'mean');
[~, false_goals_out] = rmoutliers(ALL_FALSE_GOALS, 'mean');

outliers = error_out | covar_out | goals_out | path_out | cost_out | false_goals_out;
data = ~outliers;
ALL_ERROR = ALL_ERROR(data,:);
ALL_COVAR = ALL_COVAR(data,:);
ALL_GOALS = ALL_GOALS(data,:);
ALL_PATH = ALL_PATH(data,:);
ALL_COST = ALL_COST(data,:);
ALL_FALSE_GOALS = ALL_FALSE_GOALS(data,:);

% get max, min, and mean
MIN_COST  = min(ALL_COST);
MIN_ERROR = min(ALL_ERROR);
MIN_COVAR = min(ALL_COVAR);
MIN_GOALS = min(ALL_GOALS);
MIN_PATH  = min(ALL_PATH);
MIN_FALSE_GOALS = min(ALL_FALSE_GOALS);

MEDIAN_COST  = mean(ALL_COST);
MEDIAN_ERROR = mean(ALL_ERROR);
MEDIAN_COVAR = mean(ALL_COVAR);
MEDIAN_GOALS = mean(ALL_GOALS);
MEDIAN_PATH  = mean(ALL_PATH);
MEDIAN_FALSE_GOALS = median(ALL_FALSE_GOALS);

MAX_COST  = max(ALL_COST);
MAX_ERROR = max(ALL_ERROR);
MAX_COVAR = max(ALL_COVAR);
MAX_GOALS = max(ALL_GOALS);
MAX_PATH  = max(ALL_PATH);
MAX_FALSE_GOALS = max(ALL_FALSE_GOALS);

std_COST  = std(ALL_COST);
std_ERROR = std(ALL_ERROR);
std_COVAR = std(ALL_COVAR);
std_GOALS = std(ALL_GOALS);
std_PATH  = std(ALL_PATH);
std_FALSE_GOALS = std(ALL_FALSE_GOALS);

%% plotting===============================================================
%% covariance ------------------------------------------------------
figure()
% DR
  %subplot(1,3,1);
  figure()
  X = [SIZE(DR_GOAL)-.5; SIZE(DR_STATIC); SIZE(DR_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_COVAR(DR_GOAL);MEDIAN_COVAR(DR_STATIC);MEDIAN_COVAR(DR_ADAPT)];
  Y_neg = Y_mean - [MIN_COVAR(DR_GOAL); MIN_COVAR(DR_STATIC);MIN_COVAR(DR_ADAPT)];
  Y_pos = [MAX_COVAR(DR_GOAL); MAX_COVAR(DR_STATIC);MAX_COVAR(DR_ADAPT)] - Y_mean;
  
  Y_neg = [std_COVAR(DR_GOAL); std_COVAR(DR_STATIC);std_COVAR(DR_ADAPT)];
  Y_pos = Y_neg;

  errorbar(X',Y_mean',Y_neg',Y_pos', 'x-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Covariance with Dead Reckoning");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Covariance Norm [m]");
  axis([0,.6,0,16]);

 % CI
  %subplot(1,3,2); 
  figure()
  X = [SIZE(CI_GOAL)-.5; SIZE(CI_STATIC); SIZE(CI_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_COVAR(CI_GOAL);MEDIAN_COVAR(CI_STATIC);MEDIAN_COVAR(CI_ADAPT)];
  Y_neg = Y_mean - [MIN_COVAR(CI_GOAL); MIN_COVAR(CI_STATIC);MIN_COVAR(CI_ADAPT)];
  Y_pos = [MAX_COVAR(CI_GOAL); MAX_COVAR(CI_STATIC);MAX_COVAR(CI_ADAPT)] - Y_mean;

  Y_neg = [std_COVAR(CI_GOAL); std_COVAR(CI_STATIC);std_COVAR(CI_ADAPT)];
  Y_pos = Y_neg;

  errorbar(X',Y_mean',Y_neg',Y_pos', 'o-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Covariance with Covariance Intersection");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Covariance Norm [m]");
  axis([0,.6,0,16]);

 % DEKF
 %subplot(1,3,3);
 figure()
  X = [SIZE(DEKF_GOAL)-.5; SIZE(DEKF_STATIC); SIZE(DEKF_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_COVAR(DEKF_GOAL);MEDIAN_COVAR(DEKF_STATIC);MEDIAN_COVAR(DEKF_ADAPT)];
  Y_neg = Y_mean - [MIN_COVAR(DEKF_GOAL); MIN_COVAR(DEKF_STATIC);MIN_COVAR(DEKF_ADAPT)];
  Y_pos = [MAX_COVAR(DEKF_GOAL); MAX_COVAR(DEKF_STATIC);MAX_COVAR(DEKF_ADAPT)] - Y_mean;
  
  Y_neg = [std_COVAR(DEKF_GOAL); std_COVAR(DEKF_STATIC);std_COVAR(DEKF_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 's-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Covariance with Decentralized EKF");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Covariance Norm [m]");
  axis([0,.6,0,16]);

%  % CEKF
%  subplot(1,4,4); 
%   X = [SIZE(CEKF_GOAL)-.5; SIZE(CEKF_STATIC); SIZE(CEKF_ADAPT)+.5]./100;
%   Y_mean = [MEAN_COVAR(CEKF_GOAL);MEAN_COVAR(CEKF_STATIC);MEAN_COVAR(CEKF_ADAPT)];
%   Y_neg = Y_mean - [MIN_COVAR(CEKF_GOAL); MIN_COVAR(CEKF_STATIC);MIN_COVAR(CEKF_ADAPT)];
%   Y_pos = [MAX_COVAR(CEKF_GOAL); MAX_COVAR(CEKF_STATIC);MAX_COVAR(CEKF_ADAPT)] - Y_mean;
%   errorbar(X',Y_mean',Y_neg',Y_pos', '^-');
%   legend(["goal following","static rules","adaptive rules"]);
%   title("Covariance with Centralized EKF");
%   xlabel("robot density [robots/m^2]");
%   ylabel("Covariance Norm [m]");

%% MEAN ERROR ------------------------------------------------------
 figure()
 %subplot(1,3,1); 
 figure()
  X = [SIZE(DR_GOAL)-.5; SIZE(DR_STATIC); SIZE(DR_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_ERROR(DR_GOAL);MEDIAN_ERROR(DR_STATIC);MEDIAN_ERROR(DR_ADAPT)];
  Y_neg = Y_mean - [MIN_ERROR(DR_GOAL); MIN_ERROR(DR_STATIC);MIN_ERROR(DR_ADAPT)];
  Y_pos = [MAX_ERROR(DR_GOAL); MAX_ERROR(DR_STATIC);MAX_ERROR(DR_ADAPT)] - Y_mean;
  
  Y_neg = [std_ERROR(DR_GOAL); std_ERROR(DR_STATIC);std_ERROR(DR_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 'x-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Mean Error with Dead Reckoning");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Mean Error [m]");
  axis([0,.6,0,.26]);

 % CI
  %subplot(1,3,2); 
  figure()
  X = [SIZE(CI_GOAL)-.5; SIZE(CI_STATIC); SIZE(CI_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_ERROR(CI_GOAL);MEDIAN_ERROR(CI_STATIC);MEDIAN_ERROR(CI_ADAPT)];
  Y_neg = Y_mean - [MIN_ERROR(CI_GOAL); MIN_ERROR(CI_STATIC);MIN_ERROR(CI_ADAPT)];
  Y_pos = [MAX_ERROR(CI_GOAL); MAX_ERROR(CI_STATIC);MAX_ERROR(CI_ADAPT)] - Y_mean;
  
  Y_neg = [std_ERROR(CI_GOAL); std_ERROR(CI_STATIC);std_ERROR(CI_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 'o-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Mean Error with Covariance Intersection");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Mean Error [m]");
  axis([0,.6,0,.26]);

 % DEKF
 %subplot(1,3,3); 
 figure()
  X = [SIZE(DEKF_GOAL)-.5; SIZE(DEKF_STATIC); SIZE(DEKF_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_ERROR(DEKF_GOAL);MEDIAN_ERROR(DEKF_STATIC);MEDIAN_ERROR(DEKF_ADAPT)];
  Y_neg = Y_mean - [MIN_ERROR(DEKF_GOAL); MIN_ERROR(DEKF_STATIC);MIN_ERROR(DEKF_ADAPT)];
  Y_pos = [MAX_ERROR(DEKF_GOAL); MAX_ERROR(DEKF_STATIC);MAX_ERROR(DEKF_ADAPT)] - Y_mean;
  
  Y_neg = [std_ERROR(DEKF_GOAL); std_ERROR(DEKF_STATIC);std_ERROR(DEKF_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 's-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Mean Error with Decentralized EKF");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Mean Error [m]");
  axis([0,.6,0,.26]);

%  % CEKF
%  subplot(1,4,4); 
%   X = [SIZE(CEKF_GOAL)-.5; SIZE(CEKF_STATIC); SIZE(CEKF_ADAPT)+.5]./100;
%   Y_mean = [MEAN_ERROR(CEKF_GOAL);MEAN_ERROR(CEKF_STATIC);MEAN_ERROR(CEKF_ADAPT)];
%   Y_neg = Y_mean - [MIN_ERROR(CEKF_GOAL); MIN_ERROR(CEKF_STATIC);MIN_ERROR(CEKF_ADAPT)];
%   Y_pos = [MAX_ERROR(CEKF_GOAL); MAX_ERROR(CEKF_STATIC);MAX_ERROR(CEKF_ADAPT)] - Y_mean;
%   errorbar(X',Y_mean',Y_neg',Y_pos', '^-');
%   legend(["goal following","static rules","adaptive rules"]);
%   title("Mean Error with Centralized EKF");
%   xlabel("robot density [robots/m^2]");
%   ylabel("Mean Error [m]");

%% path Deviation------------------------------------------------------
  figure()
subplot(1,3,1); 
  X = [SIZE(DR_GOAL)-.5; SIZE(DR_STATIC); SIZE(DR_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_PATH(DR_GOAL);MEDIAN_PATH(DR_STATIC);MEDIAN_PATH(DR_ADAPT)];
  Y_neg = Y_mean - [MIN_PATH(DR_GOAL); MIN_PATH(DR_STATIC);MIN_PATH(DR_ADAPT)];
  Y_pos = [MAX_PATH(DR_GOAL); MAX_PATH(DR_STATIC);MAX_PATH(DR_ADAPT)] - Y_mean;
  
  Y_neg = [std_PATH(DR_GOAL); std_PATH(DR_STATIC);std_PATH(DR_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 'x-');
  legend(["goal following","static rules","adaptive rules"]);
  title("Path Deviation with dead reckoning");
  xlabel("robot density [robots/m^2]");
  ylabel("Path Deviation [m]");
  axis([0,.6,0,7]);

 % CI
  subplot(1,3,2);  
  X = [SIZE(CI_GOAL)-.5; SIZE(CI_STATIC); SIZE(CI_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_PATH(CI_GOAL);MEDIAN_PATH(CI_STATIC);MEDIAN_PATH(CI_ADAPT)];
  Y_neg = Y_mean - [MIN_PATH(CI_GOAL); MIN_PATH(CI_STATIC);MIN_PATH(CI_ADAPT)];
  Y_pos = [MAX_PATH(CI_GOAL); MAX_PATH(CI_STATIC);MAX_PATH(CI_ADAPT)] - Y_mean;
  
  Y_neg = [std_PATH(CI_GOAL); std_PATH(CI_STATIC);std_PATH(CI_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 'o-');
  legend(["goal following","static rules","adaptive rules"]);
  title("Path Deviation with Covariance Intersection");
  xlabel("robot density [robots/m^2]");
  ylabel("Path Deviation [m]");
  axis([0,.6,0,7]);

 % DEKF
 subplot(1,3,3);  
  X = [SIZE(DEKF_GOAL)-.5; SIZE(DEKF_STATIC); SIZE(DEKF_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_PATH(DEKF_GOAL);MEDIAN_PATH(DEKF_STATIC);MEDIAN_PATH(DEKF_ADAPT)];
  Y_neg = Y_mean - [MIN_PATH(DEKF_GOAL); MIN_PATH(DEKF_STATIC);MIN_PATH(DEKF_ADAPT)];
  Y_pos = [MAX_PATH(DEKF_GOAL); MAX_PATH(DEKF_STATIC);MAX_PATH(DEKF_ADAPT)] - Y_mean;
  
  Y_neg = [std_PATH(DEKF_GOAL); std_PATH(DEKF_STATIC);std_PATH(DEKF_ADAPT)];
  Y_pos = Y_neg;

  errorbar(X',Y_mean',Y_neg',Y_pos', 's-');
  legend(["goal following","static rules","adaptive rules"]);
  title("Path Deviation with Decentralized EKF");
  xlabel("robot density [robots/m^2]");
  ylabel("Path Deviation [m]");
  axis([0,.6,0,7]);

%  % CEKF
%  subplot(1,4,4);  
%   X = [SIZE(CEKF_GOAL)-.5; SIZE(CEKF_STATIC); SIZE(CEKF_ADAPT)+.5]./100;
%   Y_mean = [MEAN_PATH(CEKF_GOAL);MEAN_PATH(CEKF_STATIC);MEAN_PATH(CEKF_ADAPT)];
%   Y_neg = Y_mean - [MIN_PATH(CEKF_GOAL); MIN_PATH(CEKF_STATIC);MIN_PATH(CEKF_ADAPT)];
%   Y_pos = [MAX_PATH(CEKF_GOAL); MAX_PATH(CEKF_STATIC);MAX_PATH(CEKF_ADAPT)] - Y_mean;
%   errorbar(X',Y_mean',Y_neg',Y_pos', '^-');
%   legend(["goal following","static rules","adaptive rules"]);
%   title("Path Deviation with Centralized EKF");
%   xlabel("robot density [robots/m^2]");
%   ylabel("Path Deviation [m]");

%% Goals Reached ------------------------------------------------------
figure()
%subplot(1,3,1);  
figure()
  X = [SIZE(DR_GOAL)-.5; SIZE(DR_STATIC); SIZE(DR_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_GOALS(DR_GOAL);MEDIAN_GOALS(DR_STATIC);MEDIAN_GOALS(DR_ADAPT)];
  Y_neg = Y_mean - [MIN_GOALS(DR_GOAL); MIN_GOALS(DR_STATIC);MIN_GOALS(DR_ADAPT)];
  Y_pos = [MAX_GOALS(DR_GOAL); MAX_GOALS(DR_STATIC);MAX_GOALS(DR_ADAPT)] - Y_mean;
  
  Y_neg = [std_GOALS(DR_GOAL); std_GOALS(DR_STATIC);std_GOALS(DR_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 'x-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Goals Reached with Dead Reckoning");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Number of Goals Reached");
  axis([0,.6,0,5]);
  

 % CI
  %subplot(1,3,2); 
  figure()
  X = [SIZE(CI_GOAL)-.5; SIZE(CI_STATIC); SIZE(CI_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_GOALS(CI_GOAL);MEDIAN_GOALS(CI_STATIC);MEDIAN_GOALS(CI_ADAPT)];
  Y_neg = Y_mean - [MIN_GOALS(CI_GOAL); MIN_GOALS(CI_STATIC);MIN_GOALS(CI_ADAPT)];
  Y_pos = [MAX_GOALS(CI_GOAL); MAX_GOALS(CI_STATIC);MAX_GOALS(CI_ADAPT)] - Y_mean;
  
  Y_neg = [std_GOALS(CI_GOAL); std_GOALS(CI_STATIC);std_GOALS(CI_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 'o-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Goals Reached with Covariance Intersection");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Number of Goals Reached");
  axis([0,.6,0,5]);

 % DEKF
%subplot(1,3,3); 
figure()
  X = [SIZE(DEKF_GOAL)-.5; SIZE(DEKF_STATIC); SIZE(DEKF_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_GOALS(DEKF_GOAL);MEDIAN_GOALS(DEKF_STATIC);MEDIAN_GOALS(DEKF_ADAPT)];
  Y_neg = Y_mean - [MIN_GOALS(DEKF_GOAL); MIN_GOALS(DEKF_STATIC);MIN_GOALS(DEKF_ADAPT)];
  Y_pos = [MAX_GOALS(DEKF_GOAL); MAX_GOALS(DEKF_STATIC);MAX_GOALS(DEKF_ADAPT)] - Y_mean;
  
  Y_neg = [std_GOALS(DEKF_GOAL); std_GOALS(DEKF_STATIC);std_GOALS(DEKF_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 's-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Goals Reached with Decentralized EKF");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Number of Goals Reached");
  axis([0,.6,0,5]);

 % CEKF
%  subplot(1,4,4);  
%   X = [SIZE(CEKF_GOAL)-.5; SIZE(CEKF_STATIC); SIZE(CEKF_ADAPT)+.5]./100;
%   Y_mean = [MEAN_GOALS(CEKF_GOAL);MEAN_GOALS(CEKF_STATIC);MEAN_GOALS(CEKF_ADAPT)];
%   Y_neg = Y_mean - [MIN_GOALS(CEKF_GOAL); MIN_GOALS(CEKF_STATIC);MIN_GOALS(CEKF_ADAPT)];
%   Y_pos = [MAX_GOALS(CEKF_GOAL); MAX_GOALS(CEKF_STATIC);MAX_GOALS(CEKF_ADAPT)] - Y_mean;
%   errorbar(X',Y_mean',Y_neg',Y_pos', '^-');
%   legend(["goal following","static rules","adaptive rules"]);
%   title("Goals Reached  with Centralized EKF");
%   xlabel("robot density [robots/m^2]");
%   ylabel("number of goals reached");

%% False Goals Reached ------------------------------------------------------
figure()
%subplot(1,3,1);  
figure()
  X = [SIZE(DR_GOAL)-.5; SIZE(DR_STATIC); SIZE(DR_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_FALSE_GOALS(DR_GOAL);MEDIAN_FALSE_GOALS(DR_STATIC);MEDIAN_FALSE_GOALS(DR_ADAPT)];
  Y_neg = Y_mean - [MIN_FALSE_GOALS(DR_GOAL); MIN_FALSE_GOALS(DR_STATIC);MIN_FALSE_GOALS(DR_ADAPT)];
  Y_pos = [MAX_FALSE_GOALS(DR_GOAL); MAX_FALSE_GOALS(DR_STATIC);MAX_FALSE_GOALS(DR_ADAPT)] - Y_mean;
  
  Y_neg = [std_FALSE_GOALS(DR_GOAL); std_FALSE_GOALS(DR_STATIC);std_FALSE_GOALS(DR_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 'x-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Goal Success Rate with Dead Reckoning");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Goal Sucess Rate [%]");
  axis([0,.6,0,100]); % 4
  

 % CI
  %subplot(1,3,2); 
  figure()
  X = [SIZE(CI_GOAL)-.5; SIZE(CI_STATIC); SIZE(CI_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_FALSE_GOALS(CI_GOAL);MEDIAN_FALSE_GOALS(CI_STATIC);MEDIAN_FALSE_GOALS(CI_ADAPT)];
  Y_neg = Y_mean - [MIN_FALSE_GOALS(CI_GOAL); MIN_FALSE_GOALS(CI_STATIC);MIN_FALSE_GOALS(CI_ADAPT)];
  Y_pos = [MAX_FALSE_GOALS(CI_GOAL); MAX_FALSE_GOALS(CI_STATIC);MAX_FALSE_GOALS(CI_ADAPT)] - Y_mean;
  
  Y_neg = [std_FALSE_GOALS(CI_GOAL); std_FALSE_GOALS(CI_STATIC);std_FALSE_GOALS(CI_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 'o-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Goal Success Rate with Covariance Intersection");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Goal Sucess Rate [%]");
  axis([0,.6,0,100]);

 % DEKF
%subplot(1,3,3); 
figure()
  X = [SIZE(DEKF_GOAL)-.5; SIZE(DEKF_STATIC); SIZE(DEKF_ADAPT)+.5]./100;
  Y_mean = [MEDIAN_FALSE_GOALS(DEKF_GOAL);MEDIAN_FALSE_GOALS(DEKF_STATIC);MEDIAN_FALSE_GOALS(DEKF_ADAPT)];
  Y_neg = Y_mean - [MIN_FALSE_GOALS(DEKF_GOAL); MIN_FALSE_GOALS(DEKF_STATIC);MIN_FALSE_GOALS(DEKF_ADAPT)];
  Y_pos = [MAX_FALSE_GOALS(DEKF_GOAL); MAX_FALSE_GOALS(DEKF_STATIC);MAX_FALSE_GOALS(DEKF_ADAPT)] - Y_mean;
 
  Y_neg = [std_FALSE_GOALS(DEKF_GOAL); std_FALSE_GOALS(DEKF_STATIC);std_FALSE_GOALS(DEKF_ADAPT)];
  Y_pos = Y_neg;
  
  errorbar(X',Y_mean',Y_neg',Y_pos', 's-');
  legend(["Goal Following","Static Rules","Adaptive Rules"]);
  title("Goal Success Rate with Decentralized EKF");
  xlabel("Robot Density [robots/m^2]");
  ylabel("Goal Sucess Rate [%]");
  axis([0,.6,0,100]);
%% -------------------------------------- RELATIVE PLOTS -------------------


% relative plot for mean covariance and mean error
figure()
for p = 1:length(MEDIAN_COST)
plot(MEDIAN_COVAR(p),MEDIAN_ERROR(p),ESTIMATOR(p), 'COLOR',RULE_COLOR(:,p),'MarkerSize',SIZE(p))
%errorbar(MEAN_COVAR(p),MEAN_ERROR(p),MEAN_ERROR(p)-MIN_ERROR(p),MAX_ERROR(p)-MEAN_ERROR(p),MEAN_COVAR(p)-MIN_COVAR(p),MAX_COVAR(p)-MEAN_COVAR(p),ESTIMATOR(p), 'COLOR',COLOR(:,p),'MarkerSize',SIZE(p))
hold on;
end
xlabel('Covariance Norm [m^2]')
ylabel('Mean Error [m]')
axis([0,14,0,.25]);

% relative plot for path deviation and goals reached
figure()
for p = 1:length(MEDIAN_COST)
plot(MEDIAN_PATH(p),MEDIAN_GOALS(p),ESTIMATOR(p), 'COLOR',RULE_COLOR(:,p),'MarkerSize',SIZE(p))
hold on;
end
xlabel('mean path deviation [m]')
ylabel('Number of goals reached')
axis([0,6,0,5]);






