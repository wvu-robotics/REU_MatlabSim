function [COST,GAINS] = kc_parfor(estimator,boids_rules,enviorment,headless,num_agents,gains)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Ka = gains(1);
Ks = gains(2);
Kc = gains(3);
Kh = gains(4);
Kg = gains(5);

COST = [];
GAINS = [];
parfor Kc = 1:5
    
[cost2,gains2] = kh_parfor(estimator,boids_rules,enviorment,headless,num_agents,[Ka;Ks;Kc;Kh;Kg])
%UNTITLED Summary of this function goes here

 %record the results for this experiment itteration
 COST = [COST, cost2];
 GAINS = [GAINS,gains2];

end
end