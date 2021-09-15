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
                   ESTIMATOR = [ESTIMATOR, '^']
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

%% plotting
MEAN_COST = mean(ALL_COST);
MEAN_ERROR = mean(ALL_ERROR);
MEAN_COVAR = mean(ALL_COVAR);

% figure()
% plot([5,10,25], b0_DR_cost, 'r*-')
% hold on;
% plot([5,10,25], b2_DR_cost, 'b*-')
% hold on;
% plot([5,10,25], b0_CI_cost, 'r^-')
% hold on;
% plot([5,10,25], b2_CI_cost, 'b^-')
% 
% yaxis(0,8)
% title('Proposed Algorithm Vs. Direct Goal following')
% xlabel('Number of Agents')
% ylabel('Goals Reached')
% legend('Direct Goal w/ Dead Reckoning', 'Adaptive Boids w/ Dead Reckoning', 'Direct Goal w/ Covariance Intersection', 'Adaptive Boids w/ Covariance Intersection')

figure()
for p = 1:length(MEAN_COST)
plot(MEAN_COVAR(p),MEAN_ERROR(p),ESTIMATOR(p), 'COLOR',COLOR(:,p),'MarkerSize',SIZE(p))
hold on;
end

xlabel('Covariance [m^2]')
ylabel('Mean Error [m]')

%figure();

% figure();
% %subplot(2,3,2)
% boxplot(ALL_COST,NAMES)
% ylabel('COST')
% %subplot(2,3,3)
% figure();
% boxplot(ALL_ERROR,NAMES)
% ylabel('Mean Error')
% %subplot(2,3,4)
% figure();
% boxplot(ALL_COVAR,NAMES)
% ylabel('Covariance')
% %subplot(2,3,5)
% figure();
% boxplot(ALL_PATH,NAMES)
% ylabel('Path Deviation')
% %subplot(2,3,6)
% figure();
% boxplot(ALL_GOALS,NAMES)
% ylabel('Goals Reached')




