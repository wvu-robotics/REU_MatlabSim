clear;
clc;
close all;

% Create and define states and actions for an MDP

MDP = createMDP(["Idle","On Alert","Protect","Defend","Search","Gather","Crashed"],["Driving","Not Driving","Detect"]);

% All possible transitions and rewards for State 1.

MDP.T(1,2,3) = 1; % From state 1 to 2 due to action 3
MDP.R(1,2,3) = 5; % Reward 
MDP.T(1,[5 6 7],1) = [0.45 0.45 0.10]; % From state 1 to 5,6,7 because of action 1
MDP.R(1,[5 6 7],1) = [3 3 -3]; % Reward

% All possible transitions and rewards for State 2.

MDP.T(2,1,2) = 1; % From state 2 to 1 due to action 2
MDP.R(2,1,2) = 1; % Reward
MDP.T(2,[3 4 5 6],3) = [0.25 0.25 0.25 0.25]; % From state 2 to 3,4 due to action 3
MDP.R(2,[3 4 5 6],3) = [7 7 3 3]; % Reward
MDP.T(2,7,1) = 1; % From state 2 to 7 due to action 1
MDP.R(2,7,1) = -3; % Reward

% All possible transitions and rewards for State 3.

MDP.T(3,1,2) = 1; % From state 3 to 1 due to action 2
MDP.R(3,1,2) = 1; % Reward
MDP.T(3,[2 4 5 6],3) = [0.25 0.25 0.25 0.25]; % From state 3 to 2,4,5,6 due to action 3
MDP.R(3,[2 4 5 6],3) = [5 7 3 3]; % Reward
MDP.T(3,7,1) = 1; % From state 3 to 7 due to action 1
MDP.R(3,7,1) = -3; % Reward

% All possible transitions and rewards for State 4.

MDP.T(4,1,2) = 1; % From state 4 to 1 due to action 2
MDP.R(4,1,2) = 1; % Reward
MDP.T(4,[2 3 5 6],3) = [0.25 0.25 0.25 0.25]; % From state 4 to 2,3,5,6 due to action 3
MDP.R(4,[2 3 5 6],3) = [5 7 3 3]; % Reward
MDP.T(4,7,1) = 1; % From state 4 to 7 due to action 1
MDP.R(4,7,1) = -3; % Reward

% All possible transitions and rewards for State 5.

MDP.T(5,1,2) = 1; % From state 5 to 1 due to action 1
MDP.R(5,1,2) = 1; % Reward
MDP.T(5,[2 3 4],3) = [0.34 0.33 0.33]; % From state 5 to 2,3,4 due to action 3
MDP.R(5,[2 3 4],3) = [5 7 7]; % Reward
MDP.T(5,[6 7],1) = [0.90 0.10]; % From state 5 to 6,7 due to action 1
MDP.R(5,[6 7],1) = [3 -3]; % Reward

% All possible transitions and rewards for State 6.

MDP.T(6,1,2) = 1; % From state 6 to 1 due to action 2
MDP.R(6,1,2) = 1; % Reward
MDP.T(6,[2 3 4 5],3) = [0.25 0.25 0.25 0.25]; % From state 6 to 2,3,4,5 due to action 3
MDP.R(6,[2 3 4 5],3) = [5 7 7 3]; % Reward
MDP.T(6,7,1) = 1; % From state 6 to 7 due to action 1
MDP.R(6,7,1) = -3; % Reward

% All possible transitions and rewards for State 7.

MDP.T(7,1,2) = 1; % From state 7 to 1 due to action 2
MDP.R(7,1,2) = 1; % Reward
MDP.T(7,2,3) = 1; % From state 7 to 2 due to action 3
MDP.R(7,2,3) = 5; % Reward
MDP.T(7,[3 4 5 6],1) = [0.25 0.25 0.25 0.25]; % From state 7 to 3,4,5,6 due to action 1
MDP.R(7,[3 4 5 6],1) = [7 7 3 3]; % Reward

% Terminal states
MDP.TerminalStates = ["Idle";"On Alert";"Protect";"Defend";"Search";"Gather";"Crashed"];

