% monte_carlo_boid_gain

%% Parameter Key ==========================================================
%     estimator :     [int]   0 = dead reckoning
%                             1 = covariance intersection
%                             2 = decentralized EKF
%                             3 = Centralized EKF
%
%     boids_rules :   [int]   0 = Goal only
%                             1 = Static rules
%                             2 = Adaptive rules
%                             3 = Adaptive rules with beacons ???
%
%     enviorment :    [int]   0 = random waypoint generator
%                             1 = foraging senario
%
%     headless :      [int]   0 = use display functions
%                             1 = do not display
%
%     num_agents :    [int]   5 = low number of agents
%                            10 = moderate number of agents
%                            25 = high number of agents
%                            50 = very high number of agents

%% SAVE DATA ???????????
save_data = 1; % if 0 then DO NOT SAVE DATA
% if 1 then SAVE DATA

%% estimator parameters
itterations = 1; %number of itterations per experiment
headless = 1;

%% run experiments

%runs every permuation of the specified experiment parameters

for num_agents = [5,10,25,50]    %[5,10,25,50] range of number of agent experiments
    for enviorment = 0   % [0,1] enviroment type to use
        for boids_rules = [1]   %[0,1,2] boids rules to use
            for estimator = [0,1,2]  %[0,1,2,3] all estimators to use
                %% file name generator
                %creates the file path to save the data to
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

                file_name = "gains/" + est + rule + env + " " + num + ".mat"

                %% result parameters
                % arrays holding all the values from every itteration of
                % each experiment
                COST = [];
                GAINS = [];
                Kg = 1;

                %% static gains
                % [Ka,Ks,Kc,Kh,Kg]
                ind = 1;
                for Ka = 0.1:.1:1
                    for Ks = 0.1:.1:1
                        for Kc = 0.1:.1:1
                            for Kh = 0.1:.1:1
                                GAINS(1,ind) = Ka;
                                GAINS(2,ind) = Ks;
                                GAINS(3,ind) = Kc;
                                GAINS(4,ind) = Kh;
                                GAINS(5,ind) = Kg;
                                ind = ind+1;
                            end
                        end
                    end
                end

                COST = zeros(length(GAINS),1);
                %% simulations
                for g = 1:length(GAINS)
                    g
                    gains = GAINS(:,g);

                    %run the experiment
                    [cost, ~, ~, ~, ~, ~] = ...
                        experiments(estimator,boids_rules,enviorment,headless,num_agents,gains);

                    %record the results for this experiment itteration
                    COST(g) = cost;
                end

                %% Plot and save data
                if save_data == 1
                    save(file_name,'COST','GAINS');
                end


            end
        end
    end
end
