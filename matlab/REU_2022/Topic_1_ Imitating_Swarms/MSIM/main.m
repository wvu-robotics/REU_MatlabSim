%% sim main script

clear ; 
close all; 
clc;

%% Simulation Parameters
sim.N=8; % number of Agents
range = 15; %radius for each agent where it 'sees' other agents
show_range=1; %activates the red circles in the figure
sim.dt = .5; % time step size
sim.map=[-100 100 -100 100];
sim.simulationTime=250;   %simulation duration
sim.accumulatedTime=0;  %comulative time
sim.initdistance = 15;   %distance between each pair of agents initial positions

 %% Initialize swarm
 AGENTS = create_swarm(sim.N,sim.initdistance,range); 
 %Assign home and dt to each agent, can include other properties. 
 for idx=1:sim.N
    AGENTS(idx).dt = sim.dt;
    AGENTS(idx).home=[0,0];
 end
 
figure()
tic
 
%% Simulation
disp('Simulation Running:');
sim.i=2;
while sim.accumulatedTime < sim.simulationTime  
    
    % take action and update
    for r = 1:sim.N
          
        % enviorment act on robot
        AGENTS(r) = AGENTS(r).update(); 
        AGENTS(r) = AGENTS(r).bounds(sim.map); 
        AGENTS(r)=AGENTS(r).get_neighbors(AGENTS);
        % robot's velocity controller
        AGENTS(r) = AGENTS(r).swarm(AGENTS(AGENTS(r).neighbors)); %agents react to enviroment and other agnets given rules in swarm
       
        
    end
    
    % show swarm and increase time
    disp(sim.accumulatedTime);
    show_swarm(AGENTS,range,show_range,sim.map);
    pause(.003);
    %Update Simulation Variables
    sim.i = sim.i + 1;
    sim.accumulatedTime = sim.accumulatedTime + sim.dt;
end


 
 
 