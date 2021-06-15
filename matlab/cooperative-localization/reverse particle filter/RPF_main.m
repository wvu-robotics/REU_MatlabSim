addpath 'C:\Users\Trevs\Desktop\github\REU_MatlabSim\matlab\cooperative-localization'

% Boids RPF_sim

%% hyper parameters ------------------------------------------------------

numBots = 10;       % number of robots in the world
spawn_len = 10;     % side length of spawning range, centered around (0,0)
time = 10;          % total time steps to run simulation for
noise = .1;         % variaince of the gaussian noise to apply to laser sensors
range = 10;         % radius of local detection range of the robots

ROBOTS = create_swarm(numBots,spawn_len); %create an array of Boids models for the robots

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
        ROBOTS(r).neighbors = neighbors;
    end
    %display the current state of the swarm
     disp_swarm(ROBOTS);
     pause(.2)
     
     %update the position of the robots and their boids rules
     for r = 1:numBots
        ROBOTS(r)= ROBOTS(r).update(); 
            
     end
     
end



