%% adaptive boids rule simulator


clear all; 
close all; 
clc;
rng(2)
% s = rng;    %save seed for testing later
addpath 'C:\Users\Trevs\Desktop\github\REU_MatlabSim\matlab\cooperative-localization'
addpath 'C:\Users\Trevs\Desktop\github\REU_MatlabSim\matlab\boids-model-master'

%% Simulation Parameters
simu.N=5;    %number of UAVs
simu.subN=8;  %subgroup size (4, 8, 16)
simu.fullCommunication=1;    %1: complete communication 0: pairwise communication

simu.simulationTime=60*60;   %flight duration (second)
simu.accumulatedTime=0;  %first timestep
simu.Ts=0.1;    %dead reckoning update rate (second)
simu.updateTs=0.1;  %ranging measurements update rate and information exchange rate (second)

simu.sigmaVelocity=0.02;  %standard deviation of velocity errors (m/s)
simu.sigmaYawRate=0.01*pi/180;    %standard deviation of yaw rate errors (rad/s)
simu.sigmaGamma=0.001*pi/180;    %standard deviation of group rotation error (rad)
simu.sigmaRange=.001;   %standard deviation of ranging measurement error (m)
simu.sigmaHeading = 1*pi/180; %standard deviation of heading measurment error (rad)
%the std of ranging measurement changes based on the range, sigmaRange = sigmaRangeRatio * truth_range + delay_error
simu.sigmaRangeRatio=1/10000;   %standard deviation of ranging measurement error ratio (Data comes from GNC paper)
simu.RangeDelayError = 0; %range measurement error caused by measurement delay

simu.percentNoiseDifference = 0.01; %slightly varies sigma per agent
simu.biasVelocity=0.1*simu.sigmaVelocity; %standard deviation of velocity turn on bias (m/s)
simu.biasYawRate=0.1*simu.sigmaYawRate; %standard deviation of yaw rate turn on bias (rad/s)
simu.initUncertainty=1;    %standard deviation of initial position uncertainty (m) 
simu.initoffset = 0;    %standard deviation of initial group offset, for testing robustness of the CML (m) 
simu.amplitude = 10;     %amplitude of the speed (m/s) ps: base line for velocity is 50 m/s
simu.initdistance = 5;   %distance between each pair of neighbot UAVs' initial positions


%For particle filter parameters
simu.sigmaMagnetic=10; %standard deviation of magnetic measurement error (nT)
simu.magneticMap = 1;    %1: altitude 305 m; 2: 1205 m; 3: 2705 m; 4: 3005 m
simu.npf = 10000; %number of particles in the particle filter
simu.threshold_resample = 0.5*simu.npf;	%if the number of effective particles below threshold, do resample

%For feedback control
simu.kd = 0.0005; %gain for distance between UAV's position and target line
simu.kh = 1;	%gain for angle between UAV's heading and target line's slope
simu.dt = 0.1;	%sample time
simu.l = 15;	%length between two tires (bicycle model)
simu.max_vel = inf;
simu.min_vel = -inf;
simu.max_steering = inf;
simu.min_steering = -inf;

%Delay steps caused by communication
if simu.fullCommunication == 0
    simu.delayN = ceil(3*simu.N / 8);
elseif simu.fullCommunication == 1
    simu.delayN = 0;
end

%Print Simulation Parameters
simu
 %% initialize swarm
 range = 5;
 e_max = 2;          % maximum mean localization error
 cov_max = 2;       % maximum covariance norm
 show_detection_rng = 1;  %toggles on and off the detection range circles
 v_max = 5;
 rho_max = simu.N / (pi*range^2);
 Crhro = simu.N*range*v_max*simu.simulationTime/(100*100)

 ROBOTS = create_swarm(simu.N,simu.initdistance,range); %create an array of Boids models for the robots
 
 for idx=1:simu.N
    %noise
    ROBOTS(idx).sigmaVelocity = simu.sigmaVelocity - (simu.percentNoiseDifference)*simu.sigmaVelocity + 2*rand*(simu.percentNoiseDifference)*simu.sigmaVelocity;
    ROBOTS(idx).sigmaYawRate = simu.sigmaYawRate - (simu.percentNoiseDifference)*simu.sigmaYawRate + 2*rand*(simu.percentNoiseDifference)*simu.sigmaYawRate;
    ROBOTS(idx).biasVelocity = normrnd(0,simu.biasVelocity);
    ROBOTS(idx).biasYawRate = normrnd(0,simu.biasYawRate);
    ROBOTS(idx).sigmaRange = simu.sigmaRange;
    ROBOTS(idx).sigmaHeading = simu.sigmaHeading;
 end
 
 %give the robots home and goal location
for r = 1:simu.N
    ROBOTS(r).home = [0,0];
    ROBOTS(r).goal = [10*rand(1,1)+35, 10*rand(1,1)-35];
    ROBOTS(r).Kg = 1;
    ROBOTS(r).found_goal = 0;
end

figure()  %object to display the simulator

 
 tic
%% Simulation
disp('Performing simulation...');
simu.i=2;
while simu.accumulatedTime < simu.simulationTime
  
    for r = 1:simu.N
    % get the lidar data (distance and bearing) to every robot
            %currently cheezy as we return values for all the robots
            %even if out of range
            % dist has noise
            % angles has noise + heading error 
     [dists,angles] = measure_dist_heading(ROBOTS,r);
        ROBOTS(r).laser = dists;
        ROBOTS(r).bearing = angles;
    
      [state_particles,neighbors] = get_locations(ROBOTS,r,range);
        for neigh = neighbors
           [ROBOTS(r),ROBOTS(neigh.ID)] = ROBOTS(r).trade_color(neigh);
        end
        ROBOTS(r).particles = state_particles;
        
       %update position with a home update if in view
        ROBOTS(r) = home_update(ROBOTS(r),simu.initdistance);
      %update the boids parameters
       ROBOTS(r) = boids_update(ROBOTS(r),e_max, rho_max);
       % determine if the robot becomes a beacon or not
       ROBOTS = beacon_update(ROBOTS,r, neighbors, cov_max);
    end
    
     
     %update the position of the robots and their boids rules
     for r = 1:simu.N
        ROBOTS(r) = ROBOTS(r).flock(neighbors); %applies acceleration to robot
        ROBOTS(r)= ROBOTS(r).update();
         if ROBOTS(r).found_goal == 1
            ROBOTS(r).goal = [ROBOTS(r).goal(1),-ROBOTS(r).goal(2)];%[100*rand(1,1)-50, 100*rand(1,1)-50];
            ROBOTS(r).Kg = 1;
            ROBOTS(r).found_goal = 0;
        end
        
     end
    
     %display the current state of the swarm
     disp_swarm(ROBOTS,range,show_detection_rng);
     pause(.03);
 %Update Simulation Variables
    simu.i = simu.i + 1;
    simu.accumulatedTime = simu.accumulatedTime + simu.Ts;
end
 
 
















