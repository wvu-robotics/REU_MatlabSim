<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> 89c2170d7a65af453fd49c91cfd39ae74b8c174a
=======
>>>>>>> 89c2170d7a65af453fd49c91cfd39ae74b8c174a
clc
clear
close all
global CURRENT_KEY_PRESSED 
CURRENT_KEY_PRESSED = '';
H = figure;
set(H,'KeyPressFcn',@buttonPress);
<<<<<<< HEAD
<<<<<<< HEAD
rosinit('10.255.96.126');
=======
rosinit('10.255.103.55');
>>>>>>> 89c2170d7a65af453fd49c91cfd39ae74b8c174a
=======
rosinit('10.255.103.55');
>>>>>>> 89c2170d7a65af453fd49c91cfd39ae74b8c174a
numberOfAgents = 1;
env = agentEnv(numberOfAgents,@rosController,10,.1);
env.setAgentPositions(zeros(numberOfAgents, 2));
env.setGoalPositions([5, 5]);
env.agents(1).setUpPublisher('/turtle1/cmd_vel/');
env.agents(1).setUpSubscriber('/turtle1/cmd_vel/');
counter = 0;
ENV.realTime = true;
while true 
   counter = counter + 1;
   env.tickRos;
end
rosshutdown;

function buttonPress(src,event)
  global CURRENT_KEY_PRESSED
  CURRENT_KEY_PRESSED = event.Key;
end