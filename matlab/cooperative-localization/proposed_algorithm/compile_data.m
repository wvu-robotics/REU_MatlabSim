%% compiles all the data from generate data to plot the results

%needs alot of work and adjustmet to display all the data in a good clean
%way

%% results matricies
NAMES = [];
ALL_ERROR = [];
ALL_COVAR = [];
ALL_GOALS = [];
ALL_COST = [];
ALL_PATH = [];

b0_DR_cost = [];
b0_CI_cost = [];
b2_CI_cost= [];
b2_DR_cost = [];
X = [];
N = [];
COLOR = [];
ESTIMATOR = [];
SIZE = [];

%% data selection
% each of these for loops specififes which data you want to load to see
for estimator = [0,1,2] %[0,1,2] 
    for enviorment = 0
        for boids_rules = [0,1,2]
            for num_agents = [5,10,25] %[5,10,25,50]
                           
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
                
                %NAMES = [NAMES,file_name];
                ALL_ERROR = [ALL_ERROR,MEAN_ERROR'];
                ALL_COVAR = [ALL_COVAR,COVAR'];
                ALL_GOALS = [ALL_GOALS,GOALS_REACHED'];
                ALL_COST = [ALL_COST,COST'];
                ALL_PATH = [ALL_PATH,PATH_DEVIATION'];  
                X = [X,boids_rules];
                N = [N,num_agents];
                    
%                 if boids_rules == 0 && estimator == 0
%                     b0_DR_cost = [b0_DR_cost,mean(GOALS_REACHED)];
%                 elseif boids_rules == 2 && estimator == 0
%                     b2_DR_cost = [b2_DR_cost,mean(GOALS_REACHED)];
%                 elseif boids_rules == 0 && estimator == 1
%                     b0_CI_cost = [b0_CI_cost,mean(GOALS_REACHED)];
%                 elseif boids_rules == 2 && estimator == 1
%                     b2_CI_cost = [b2_CI_cost,mean(GOALS_REACHED)];
%                 end
                
                if boids_rules == 0
                    COLOR = [COLOR, [1;0;0]];
                elseif boids_rules == 1
                    COLOR = [COLOR, [0;1;0]];
                else
                   COLOR = [COLOR, [0;0;1]]; 
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
                
            end
        end
    end
end

%% metrics
MEAN_COST  = mean(ALL_COST);
MEAN_ERROR = mean(ALL_ERROR);
MEAN_COVAR = mean(ALL_COVAR);
MEAN_GOALS = mean(ALL_GOALS);
MEAN_PATH  = mean(ALL_PATH);

MIN_COST  = min(ALL_COST);
MIN_ERROR = min(ALL_ERROR);
MIN_COVAR = min(ALL_COVAR);
MIN_GOALS = min(ALL_GOALS);
MIN_PATH  = min(ALL_PATH);

MAX_COST  = max(ALL_COST);
MAX_ERROR = max(ALL_ERROR);
MAX_COVAR = max(ALL_COVAR);
MAX_GOALS = max(ALL_GOALS);
MAX_PATH  = max(ALL_PATH);

%% plotting





%-------------------------------------- RELATIVE PLOTS -------------------

% relative plot for mean covariance and mean error
figure()
for p = 1:length(MEAN_COST)
plot(MEAN_COVAR(p),MEAN_ERROR(p),ESTIMATOR(p), 'COLOR',COLOR(:,p),'MarkerSize',SIZE(p))
%errorbar(MEAN_COVAR(p),MEAN_ERROR(p),MEAN_ERROR(p)-MIN_ERROR(p),MAX_ERROR(p)-MEAN_ERROR(p),MEAN_COVAR(p)-MIN_COVAR(p),MAX_COVAR(p)-MEAN_COVAR(p),ESTIMATOR(p), 'COLOR',COLOR(:,p),'MarkerSize',SIZE(p))
hold on;
end
xlabel('Covariance Norm [m^2]')
ylabel('Mean Error [m]')

% relative plot for path deviation and goals reached
figure()
for p = 1:length(MEAN_COST)
plot(MEAN_PATH(p),MEAN_GOALS(p),ESTIMATOR(p), 'COLOR',COLOR(:,p),'MarkerSize',SIZE(p))
hold on;
end
xlabel('mean path deviation [m]')
ylabel('Number of goals reached')







