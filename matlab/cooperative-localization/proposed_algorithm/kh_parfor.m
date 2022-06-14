function [COST,GAINS] = kh_parfor(estimator,boids_rules,enviorment,headless,num_agents,gains)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Ka = gains(1);
Ks = gains(2);
Kc = gains(3);
Kh = gains(4);
Kg = gains(5);
COST = [];
GAINS = [];

parfor Kh = 1:5
    num_agents
    gains
[cost, ~, ~, ~, ~, ~] = ...
 experiments(estimator,boids_rules,enviorment,headless,num_agents,[Ka;Ks;Kc;Kh;Kg]);

 %record the results for this experiment itteration
 COST = [COST, cost];
 GAINS = [GAINS,[Ka;Ks;Kc;Kh;Kg]];

end
end