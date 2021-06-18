addpath 'C:\Users\Trevs\Desktop\github\REU_MatlabSim\matlab\cooperative-localization'
addpath 'C:\Users\Trevs\Desktop\github\REU_MatlabSim\matlab\boids-model-master'
% Boids RPF_sim

%% hyper parameters ------------------------------------------------------

numBots = 50;       % number of robots in the world
spawn_len = 10;     % side length of spawning range, centered around (0,0)
time = 10000;          % total time steps to run simulation for
noise = .5;         % variaince of the gaussian noise to apply to laser sensors
range = 10;         % radius of local detection range of the robots
e_max = 2;          % maximum mean localization error
show_detection_rng = 0;  %toggles on and off the detection range circles

%% initialize swarm-------------------------------------------------------
ROBOTS = create_swarm(numBots,spawn_len,range); %create an array of Boids models for the robots

rho_max = numBots / (pi*range^2);

%give the robots home location
for r = 1:numBots
    ROBOTS(r).home = [0,0];
end

figure()  %object to display the simulator

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
        ROBOTS(r).particles = state_particles;
        %update position with a home update if in view
        ROBOTS(r) = home_update(ROBOTS(r),spawn_len);
        %update the boids parameters
        ROBOTS(r) = boids_update(ROBOTS(r),e_max, rho_max);
    end
    %display the current state of the swarm
     disp_swarm(ROBOTS,range,show_detection_rng);
     pause(.01)
     
     %update the position of the robots and their boids rules
     for r = 1:numBots
         
        ROBOTS(r) = ROBOTS(r).flock(neighbors);
        ROBOTS(r)= ROBOTS(r).update(); 
        
     end
     
end



