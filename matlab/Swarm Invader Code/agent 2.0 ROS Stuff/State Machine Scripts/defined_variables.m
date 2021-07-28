% This script is to be run in the launch file. It will create variables in
% the base workspace, which is necessary when creating the separate
% proeprties for each individual agent. 
battery_life = 100;
current_state = 0; % States are listed below:
                        % Idle = 0                      ; w = $1
                        % On Alert = 1                  ; w = $2
                        % In Pursuit = 2                ; w = $5
                        % Deciding = 3                  ; w = $1
                        % Returning Home = 4            ; w = $1
                        % Protecting Home = 5           ; w = $2
                        % Charging = 6                  ; w = $1
                        % Searching = 7                 ; w = $2
                        % Gathering = 8                 ; w = $5
                        % Shut-down (battery = 1%) = 9  ; w = -$2
% iteration = 0;
wallet = 0; % This will start them out with $0
charging_iterations = 0;
searching_iterations = 0;
gathering_iterations = 0;
deciding_number = 0;
outstanding_distance = 0;


initial_number = 0;
invader_detected = 0;
distance_from_home = 0;
distance_from_invader = 0;
enough_battery_home = 0;
enough_battery_invader = 0;
enough_battery_home_invader = 0;

% Any questions can be directed to danielvillarrealusa@gmail.com.