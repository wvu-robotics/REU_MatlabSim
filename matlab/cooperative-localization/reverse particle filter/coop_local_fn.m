function [avg_mean_error,avg_covar,goals_per_robot] = coop_local_fn(W,visualize)



addpath 'C:\Users\Trevs\Desktop\github\REU_MatlabSim\matlab\cooperative-localization'
addpath 'C:\Users\Trevs\Desktop\github\REU_MatlabSim\matlab\boids-model-master'
% Boids RPF_sim

%% hyper parameters ------------------------------------------------------

numBots = 20;       % number of robots in the world
spawn_len = 5;     % side length of spawning range, centered around (0,0)
time = 100;          % total time steps to run simulation for
noise = .1;         % variaince of the gaussian noise to apply to laser sensors
range = 5;         % radius of local detection range of the robots
e_max = 2;          % maximum mean localization error
show_detection_rng = 0;  %toggles on and off the detection range circles

%% setup error parameters
MEAN_ERROR = [];
COVAR_ERROR = [];
GOALS_REACHED = 0;

%% initialize swarm-------------------------------------------------------
ROBOTS = create_swarm(numBots,spawn_len,range); %create an array of Boids models for the robots

rho_max = numBots / (pi*range^2);

%give the robots home and goal location
for r = 1:numBots
    ROBOTS(r).home = [0,0];
    ROBOTS(r).goal = [10*rand(1,1)+35, 10*rand(1,1)-35];
    ROBOTS(r).Kg = 1;
    ROBOTS(r).found_goal = 0;
end

if visualize == 1
    figure()  %object to display the simulator
end

%% --------------------------------simulaate the robots
for t = 1:time
    for r = 1:numBots
       % get the lidar data (distance and bearing) to every robot
                %currently cheezy as we return values for all the robots
                %even if out of range
        [dists,angles] = measure_dist_heading(ROBOTS,r,noise);
        ROBOTS(r).laser = dists;
        ROBOTS(r).bearing = angles;
        
        %based off the robots in range find neighbor robot objects and
        %recieve their dead reckogning states
        [state_particles,neighbors] = get_locations(ROBOTS,r,range);
        for neigh = neighbors
            ROBOTS(r).trade_color(neigh);
        end
        ROBOTS(r).particles = state_particles;
        %update position with a home update if in view
        ROBOTS(r) = home_update(ROBOTS(r),spawn_len);
        %update the boids parameters
       ROBOTS(r) = boids_gain(ROBOTS(r),W);
    end
    %display the current state of the swarm
     
    
    if visualize == 1
        disp_swarm(ROBOTS,range,show_detection_rng);
        pause(.1)
    end
     
     %update the position of the robots and their boids rules
     for r = 1:numBots
        MEAN_ERROR(r,t) = norm(ROBOTS(r).mean_position - ROBOTS(r).position);
        COVAR_ERROR(r,t) = norm(ROBOTS(r).mean_covar);
        
        ROBOTS(r) = ROBOTS(r).flock(neighbors);
        ROBOTS(r).velocity = ROBOTS(r).velocity/norm(ROBOTS(r).covariance);
        ROBOTS(r)= ROBOTS(r).update(noise); 
        ROBOTS(r).covariance = ROBOTS(r).covariance + [noise,.01;.01,noise];
        if ROBOTS(r).found_goal == 1
            ROBOTS(r).goal = [100*rand(1,1)-50, 100*rand(1,1)-50];
            ROBOTS(r).Kg = 1;
            ROBOTS(r).found_goal = 0;
            GOALS_REACHED = GOALS_REACHED +1;
        end
        
     end
     
end

avg_mean_error = mean(mean(MEAN_ERROR));
avg_covar = mean(mean(COVAR_ERROR));
goals_per_robot = GOALS_REACHED/numBots;


end

