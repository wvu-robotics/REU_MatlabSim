
clc
clear
close all
global CURRENT_KEY_PRESSED 
CURRENT_KEY_PRESSED = '';
H = figure;
set(H,'KeyPressFcn',@buttonPress);
numberOfAgents = 4;
env = agentEnv(numberOfAgents,@rosController,2,.01);
env.agents(1).setShape(.25*[-1,-1; -1, 1; 1,0]);
env.agents(2).setShape(.25*[-1,-1; -1, 1; 1,0]);
env.agents(3).setShape(.25*[-1,-1; -1, 1; 1,0]);
env.agents(4).setShape(.25*[-1,-1; -1, 1; 1,0]);

for i = 1:numberOfAgents
   env.agents(i).createProperty('enemy', false); 
end
env.agents(1).setProperty('enemy', true);

env.setAgentPositions(zeros(numberOfAgents, 2));G
env.setGoalPositions(zeros(numberOfAgents, 2));

env.agents(1).setUpPublisher('/turtle2/cmd_vel');
env.agents(1).setUpSubscriber('/vicon/turtle2/turtle2');
env.agents(2).setUpPublisher('/turtle3/cmd_vel');
env.agents(2).setUpSubscriber('/vicon/turtle3/turtle3');
env.agents(3).setUpPublisher('/turtle4/cmd_vel');
env.agents(3).setUpSubscriber('/vicon/turtle4/turtle4');
env.agents(4).setUpPublisher('/turtle5/cmd_vel');
env.agents(4).setUpSubscriber('/vicon/turtle5/turtle5');
counter = 0;
env.realTime = true;
env.pathVisibility(true);
while true
   counter = counter + 1;
   env.tickRos;
end
rosshutdown;

function buttonPress(src,event)
  global CURRENT_KEY_PRESSED
  CURRENT_KEY_PRESSED = event.Key;
end
