addpath 'C:\Users\dqish\Documents\GitHub\REU_MatlabSim\matlab\cooperative-localization'
addpath 'C:\Users\dqish\Documents\GitHub\REU_MatlabSim\matlab\boids-model-master'
addpath 'C:\Users\dqish\Documents\GitHub\REU_MatlabSim\matlab\cooperative-localization\reverse particle filter'
addpath 'C:\Users\dqish\Documents\GitHub\REU_MatlabSim\matlab\flow field'
% Boids RPF_sim

% hyper parameters ------------------------------------------------------

numBots = 20;       % number of robots in the world
spawn_len = 8;     % side length of spawning range, centered around (0,0)
time = 1000;          % total time steps to run simulation for
noise = .1;         % variaince of the gaussian noise to apply to laser sensors
range = 5;         % radius of local detection range of the robots
e_max = 2;          % maximum mean localization error
cov_max = 2;       % maximum covariance norm
show_detection_rng = 0;  %toggles on and off the detection range circles
v_max = 2;

% initialize swarm-------------------------------------------------------
ROBOTS = create_swarm(numBots,spawn_len,range); %create an array of Boids models for the robots

rho_max = numBots / (pi*range^2);
Crhro = range*v_max*time/(100*100);

%%give the robots home and goal location
for r = 1:numBots
    ROBOTS(r).home = [0,0];
    ROBOTS(r).goal = [10*rand(1,1)+35, 10*rand(1,1)-35];
    ROBOTS(r).Kg = 1;
    ROBOTS(r).found_goal = 0;
end
 ROBOTS(1).Isinvader=1;
figure()  %object to display the simulator

% --------------------------------simulate the robots
for t = 1:time
    for r = 2:numBots
       % get the lidar data (distance and bearing) to every robot
                %currently cheezy as we return values for all the robots
                %even if out of range
        [dists,angles] = measure_dist_heading(ROBOTS,r,noise);
        ROBOTS(r).laser = dists;
        ROBOTS(r).bearing = angles;
        [state_particles,neighbors] = get_locations(ROBOTS,r,range);
        Invaders = getinvaders(ROBOTS(r),neighbors);
        ROBOTS(r).invaders = Invaders;
        %based off the robots in range find neighbor robot objects and
        %recieve their dead reckogning states
        %[state_particles,neighbors] = get_locations(ROBOTS,r,range);
       
    end
    %display the current state of the swarm
     disp_swarm(ROBOTS,range,show_detection_rng);
     pause(.1)
     
     %update the position of the robots and their boids rules
     for r = 1:numBots
        ROBOTS(r).velocity= defendercontroller(ROBOTS(r));
        
      
        ROBOTS(r)= ROBOTS(r).update(noise); 
       
        if ROBOTS(r).found_goal == 1
            ROBOTS(r).goal = [100*rand(1,1)-50, 100*rand(1,1)-50];
            ROBOTS(r).Kg = 1;
            ROBOTS(r).found_goal = 0;
        end
     end
     
end



