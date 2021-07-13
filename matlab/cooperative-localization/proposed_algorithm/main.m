% cooperative localization main function



%% adaptive boids rule simulator

clear all; 
close all; 
clc;

%% Simulation Parameters
simu.N=3;    %number of Robotss
simu.dt = .1; % time step size
simu.simulationTime=60*60;   %flight duration (second)
simu.accumulatedTime=0;  %first timestep

simu.percentNoiseDifference =0.01; %slightly varies sigma per agent
simu.sigmaVelocity=0.02;  %standard deviation of velocity errors (m/s)
simu.sigmaYawRate=0.01*pi/180;    %standard deviation of yaw rate errors (rad/s)
simu.sigmaRange=.001;   %standard deviation of ranging measurement error (m)
simu.sigmaHeading = 1*pi/180; %standard deviation of heading measurment error (rad)
simu.biasVelocity=0.1*simu.sigmaVelocity; %standard deviation of velocity turn on bias (m/s)
simu.biasYawRate=0.1*simu.sigmaYawRate; %standard deviation of yaw rate turn on bias (rad/s)

simu.initdistance = 5;   %distance between each pair of neighbot UAVs' initial positions

 %% initialize swarm
 range = 5;
 e_max = 2;          % maximum mean localization error
 cov_max = 2;       % maximum covariance norm
 show_detection_rng = 1;  %toggles on and off the detection range circles
 v_max = 5;
 rho_max = simu.N / (pi*range^2);
 Crhro = simu.N*range*v_max*simu.simulationTime/(100*100)

 ROBOTS = create_robot_swarm(simu.N,simu.initdistance,range); %create an array of Boids models for the robots
 
 for idx=1:simu.N
    %noise
    ROBOTS(idx).sigmaVelocity = simu.sigmaVelocity - (simu.percentNoiseDifference)*simu.sigmaVelocity + 2*rand*(simu.percentNoiseDifference)*simu.sigmaVelocity;
    ROBOTS(idx).sigmaYawRate = simu.sigmaYawRate - (simu.percentNoiseDifference)*simu.sigmaYawRate + 2*rand*(simu.percentNoiseDifference)*simu.sigmaYawRate;
    ROBOTS(idx).biasVelocity = normrnd(0,simu.biasVelocity);
    ROBOTS(idx).biasYawRate = normrnd(0,simu.biasYawRate);
    ROBOTS(idx).sigmaRange = simu.sigmaRange;
    ROBOTS(idx).sigmaHeading = simu.sigmaHeading;
    ROBOTS(idx).dt = simu.dt;
 end
 
 %give the robots home and goal location and esimator
for r = 1:simu.N
    ROBOTS(r).home = [0,0];
    ROBOTS(r).goal = [10*rand(1,1)+35, 10*rand(1,1)-35];
    ROBOTS(r).Kg = 1000;
    ROBOTS(r).found_goal = 0;
    ROBOTS(r).estimator = 1; % just dead reckoning 
end

figure()  %object to display the simulator

 
 tic
 
 %% Simulation
disp('Performing simulation...');
simu.i=2;
while simu.accumulatedTime < simu.simulationTime  
   %% measure the enviorment-----------------------
   [X,P] = get_states(ROBOTS);
   for r = 1:simu.N
        % all other robot positions and covariances
        ROBOTS(r).X = X;
        ROBOTS(r).P = P;
       % ROBOTS(r) = ROBOTS(r).get_states(ROBOTS);
       
        % range and bearing
        ROBOTS(r) = ROBOTS(r).lidar_measurement(ROBOTS);
        % velocity magnitude and yaw rate
        ROBOTS(r) = ROBOTS(r).encoder_measurement(); 
        % get prediction of my location from the other robots
        [states, covars] = ROBOTS(r).get_locations(ROBOTS);
        ROBOTS(r).state_particles{1} = states;
        ROBOTS(r).state_particles{2} = covars;
        

   end
    
    %% take action and update------------------------
    for r = 1:simu.N
        
        % robot act on enviorment
        if simu.accumulatedTime ==0
            ROBOTS(r).vel_m = 0;
            ROBOTS(r).yaw_rate_m = 0;
            ROBOTS(r).estimator = 0;
        else
            ROBOTS(r).estimator = 1;
        end
        
        ROBOTS(r) = ROBOTS(r).flock(ROBOTS(ROBOTS(r).neighbors));
        
        % enviorment act on robot
        ROBOTS(r) = ROBOTS(r).update();
    end
    
    %% display and increase time-------------------------
     
    %display the current state of the swarm
     disp(simu.accumulatedTime);
     disp_swarm(ROBOTS,range,show_detection_rng);
     pause(.003);
    %Update Simulation Variables
    simu.i = simu.i + 1;
    simu.accumulatedTime = simu.accumulatedTime + simu.dt;
end
 
 
 