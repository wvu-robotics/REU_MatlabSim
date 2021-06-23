
%% -------------------------- EKF SIMULATOR -------------------------------

number_of_robots = 5; % Number of Robots
dt = 1; % Increment of Time 1 second
ROBOTS = []; % Define the struct array for all the robots
range_of_robot = 5; % Radious of the laser
amount_of_steps = 10; % Define the amount of steps each robot takes
pairwise = 2; % Limit the communication between robots each time step
robots_and_base = number_of_robots + 1; % Variable with total number of robots and the base

%% ---------------------- INITIALIZE ROBOTS & BASE-------------------------

x = 0; % Define x coordinate for base
y = 0; % Define y coordinate for base
theta = 0; % Define theta for base
speed = 0; % Define speed for base
robot = struct('pose', [x,y,theta], 'velocity', speed, 'path', [x,y,theta], 'neighbor_ID', zeros(amount_of_steps,1), 'laser', zeros(amount_of_steps,1));
% Assign values for varaibles in the struct array
ROBOTS = [ROBOTS,robot]; % Combined struct array with variables for base
for r = 1:number_of_robots % For each robot in the total number of ROBOTS
    x = 10*rand(1,1); % Define array [1,1] value from 0 to 10
    y = 10*rand(1,1); % Define array [1,1] value from 0 to 10
    theta = 2*pi*rand(1,1); % Define array [1,1] value from 0 to 2*PI
    speed = 5*rand(1,1); % Define array [1,1] value from 0 to 5
    robot = struct('pose', [x,y,theta], 'velocity', speed, 'path', [x,y,theta], 'neighbor_ID', zeros(amount_of_steps,1), 'laser', zeros(amount_of_steps,1));
    % Asign values for different varaibles in the struct array 
    ROBOTS = [ROBOTS,robot]; % Append the variables for the rest of the robots
end

%% ------------------------- SIMULATE ROBOTS ------------------------------

for t = 1:amount_of_steps % For each time (1 second) in the total time (10 seconds)
    %ROBOTS(1).pose = ROBOTS(1).pose;
    % Define the next pose of the base the same as the previous one (stationary)
    %ROBOTS(1).path = [ROBOTS(1).path; ROBOTS(1).pose];
    % Add the new position of the base to the path array
    for r = 1:robots_and_base % For each robot in the total number of ROBOTS
        ROBOTS(r).pose = ROBOTS(r).pose + [ROBOTS(r).velocity*cos(ROBOTS(r).pose(3))*dt, ROBOTS(r).velocity*sin(ROBOTS(r).pose(3))*dt,0];
        % Calculate the true new postition based on the last postion and 
        % the velocity, increment of time and angle of displacement
        ROBOTS(r).path = [ROBOTS(r).path; ROBOTS(r).pose];
        % Add the new true position to the path array
    end
    % Make all the robots move firts
    for r = 1:robots_and_base % For each robot in the total number of ROBOTS
        distance = []; % Create array for distances
        for R = 1:robots_and_base % For each robot in the total number of ROBOTS
            distance = [distance, norm(ROBOTS(R).pose(1:2) - ROBOTS(r).pose(1:2))];
            % Calculate the normalized distance from each robot to the rest
            % of the robots at each time step with no error
            if distance(R) > range_of_robot % Out of range
                distance(R) = 0; % Set equal to 0 all distances out of range
            elseif distance(R) < 0.5 % Youself
                distance(R) = 0; % Set equal to 0 your own range
            end
        end
        possible_distance = []; % Create array for possible distances
        possible_distance = find(distance); % Extract the ones that are not equal to 0
        if isempty(possible_distance) == 1 % If there is no neighbors close
           neighbor = 0; % Set the neighbor ID equal to 0
           distance_neighbor = 0; % Set the distance corresponding to that neighbor ID equal to 0
           ROBOTS(r).neighbor_ID(t)=neighbor;
           % Append no neighbour ID = 0
           ROBOTS(r).laser(t) = distance_neighbor;
           % Append no distance to that neighbor ID = 0
        else % If there is at least 1 neighbor
           range_base = ismember(1, possible_distance);
           % Determine if the robot is in range with the base
           if range_base == 1 % If the robot is in range with the base
               random = 1;  % Pick the distance from the base
               neighbor = possible_distance(random); % Extract the base ID (1)
               distance_neighbor = distance(neighbor);  % Extract the distance corresponding to that base distance
               ROBOTS(r).neighbor_ID(t)=neighbor;
               % Append the base ID
               ROBOTS(r).laser(t) = distance_neighbor;
               % Append the distance to the base with no error
           else % If the robot is not in range with the base
               random = randi(length(possible_distance)); % Pick a random distance
               neighbor = possible_distance(random); % Extract the neighbor ID
               distance_neighbor = distance(neighbor); % Extract the distance corresponding to that neighbor ID
               ROBOTS(r).neighbor_ID(t)=neighbor;
               % Append the neighbour ID
               ROBOTS(r).laser(t) = distance_neighbor + normrnd(0,.05,1,1);
               % Append the distance to that neighbor ID with some small white 
               % gaussian error measurement (mean=0 and stdv= 0.05)
           end
        end
    end
    % Calculate relative distances from each robot to the other robots,
    % select a valid neighbor (in range and not itself) and append the
    % ID and the distance to them for each time step with some gaussian error.
    % If base is in range, the rest of the neighbors are ignored
end

%% ------------------------- PLOT THRUTH DATA -----------------------------

figure()% Create new figure
axis equal % Set aspect ratio in both axis equal (Better visualization ???)
plot(ROBOTS(1).path(:,1),ROBOTS(1).path(:,2),'ko','MarkerSize',15); % Plot the base (black square)
hold on; % Hold the figure
viscircles([ROBOTS(1).path(1,1),ROBOTS(1).path(1,2)],range_of_robot,'LineStyle','--','Color','k');
for r = 2:robots_and_base % For each robot in the total number of ROBOTS
    quiver(ROBOTS(r).path(:,1),ROBOTS(r).path(:,2),ROBOTS(r).velocity*cos(ROBOTS(r).pose(3))*ones(11,1), ROBOTS(r).velocity*sin(ROBOTS(r).pose(3))*ones(11,1),0,'r');
    % Plot vector with cartesian coordinates in each direction with 0 scale
    % (red unit vectors)
    % X values, Y values, X increment vector, Y increment vector, scale
    plot(ROBOTS(r).path(:,1), ROBOTS(r).path(:,2),'ro','MarkerSize',3)
    % Plot the points after each time step
end
% Plot the robots

%% --------------------- RUN FILTER ON GENERATED DATA ---------------------

X_i = zeros(pairwise*3,1); % Initialize the current state matrix
F = zeros(pairwise*3,1); % Initialize the kinematic gain matrix
Y_m = []; % Initialize the current measurement of the state ~ LIDAR
ID = []; % Initialize the ID list to track the neighbors

% Simulate the first step (No previous knowledge - No EKF)
for r = 2:robots_and_base % For each robot in the total number of ROBOTS
     X_i(1,(r-1)) = ROBOTS(r).path(1,1);
     % Set current state matrix with x component for robot 1
     X_i(3,(r-1)) = ROBOTS(r).path(1,2);
     % Set current state matrix with y component for robot 1
     X_i(5,(r-1)) = ROBOTS(r).path(1,3);
     % Set current state matrix with theta component for robot 1
     F(1,(r-1)) = ROBOTS(r).velocity*cos(ROBOTS(r).pose(3))+ normrnd(0,.1,1,1);
     % Set kinematic gain to estimate next step for x component for robot 1
     % with small white gaussian noise (mean=0 and stdv= 0.1)
     F(3,(r-1)) = ROBOTS(r).velocity*sin(ROBOTS(r).pose(3)) + normrnd(0,.1,1,1);
     % Set kinematic gain to estimate next step for y component for robto 1
     % with small white gaussian noise (mean=0 and stdv= 0.1)
     % Already set the kinematic gain to estimate next step for theta
     % component equal to 0 for robot 1 and robot 2
     % F(5,(r-1)) = 0; 
     % F(6,(r-1)) = 0;  
    if ROBOTS(r).neighbor_ID(1)== 0 % When the robot doesn´t have neighbors
        X_i(2,(r-1)) = -1;
        % Set current state matrix with x component for robot 2
        X_i(4,(r-1)) = -1;
        % Set current state matrix with y component for robot 2
        X_i(6,(r-1)) = -1;
        % Set current state matrix with theta component for robot 2
        F(2,(r-1)) = -1;
        % Set kinematic gain to estimate next step for x component for robot 2
        % with small white gaussian noise (mean=0 and stdv= 0.1)
        F(4,(r-1)) = -1;
        % Set kinematic gain to estimate next step for y component for robot 2
        % with small white gaussian noise (mean=0 and stdv= 0.1)
    else % When there is at least one neighbor
        X_i(2,(r-1)) = ROBOTS(ROBOTS(r).neighbor_ID(1)).path(1,1);
        % Set current state matrix with x component for robot 2
        X_i(4,(r-1)) = ROBOTS(ROBOTS(r).neighbor_ID(1)).path(1,2);
        % Set current state matrix with y component for robot 2
        X_i(6,(r-1)) = ROBOTS(ROBOTS(r).neighbor_ID(1)).path(1,3);
        % Set current state matrix with theta component for robot 2
        F(2,(r-1)) = ROBOTS(ROBOTS(r).neighbor_ID(1)).velocity*cos(ROBOTS(ROBOTS(r).neighbor_ID(1)).pose(3)) + normrnd(0,.1,1,1);
        % Set kinematic gain to estimate next step for x component for robot 2
        % with small white gaussian noise (mean=0 and stdv= 0.1)
        F(4,(r-1)) = ROBOTS(ROBOTS(r).neighbor_ID(1)).velocity*sin(ROBOTS(ROBOTS(r).neighbor_ID(1)).pose(3)) + normrnd(0,.1,1,1);
        % Set kinematic gain to estimate next step for y component for robot 2
        % with small white gaussian noise (mean=0 and stdv= 0.1)
    end
end
quiver(X_i(1,:), X_i(pairwise+1,:),F(1,:), F(pairwise+1,:),0, 'b');
% Plot vector with cartesian coordinates in each direction with 0 scale
% (blue unit vectors)
% X values, Y values, X increment vector, Y increment vector, scale

P_i = cell(1,number_of_robots); % Set the current covariance matrices cell (size)
for r = 1:number_of_robots
    P_i{r} = 0.1*eye(pairwise*3); % Set the current covariance matrix for each robot
end
Q = 0.1*P_i{1}; % Set the estimation noise covariance matrix (Value ???)
R = 1*eye(pairwise*2); % Set the measurement noise covariance matrix (Value ???)
% Simulate the rest of the steps (Using EKF)
for t = 1 : amount_of_steps % For time step 2 to the rest
    for r = 2:robots_and_base % For each robot in the total number of ROBOTS
        if ROBOTS(r).neighbor_ID(t)== 0 % There is no neighbor (any -1) in the state matrix
            X_f(:,(r-1)) = X_i(:,(r-1)) + F(:,(r-1));
            % A posteriori estimation using Kinematic gain
        else % If there is any neighbor
            Y_m = ROBOTS(r).laser(t); % Append the distance to the random neighbor
            [X_f(:,(r-1)),P_f] = Decentral_swarm_EKF(X_i(:,(r-1)), P_i{r-1}, Y_m, F(:,(r-1)), dt, Q, R);
            % A posteriori estimation using EKF
            P_i{r-1} = P_f; % Set the new current covariance matrix equal to the last a posteriori estimation for the covariance matrix
        end
    end
    for r = 2:robots_and_base-1 % For each robot in the total number of ROBOTS
        % Set the new current state matrix equal to the last a posteriori estimation for the state matrix
        % Set the new current kinematic gain matrix accordingly
        X_i(1,(r-1)) = X_f(1,(r-1));
        X_i(3,(r-1)) = X_f(3,(r-1));
        X_i(5,(r-1)) = X_f(5,(r-1));
        F(1,(r-1)) = ROBOTS(r).velocity*cos(ROBOTS(r).pose(3))+ normrnd(0,.1,1,1);
        F(3,(r-1)) = ROBOTS(r).velocity*sin(ROBOTS(r).pose(3)) + normrnd(0,.1,1,1);
        % F(5,(r-1)) = 0; 
        % F(6,(r-1)) = 0;
        if t==amount_of_steps % To avoid errors in the last time step
            % Do nothing
        else
            if ROBOTS(r).neighbor_ID(t+1)== 0 % When the robot doesn´t have neighbors
                X_i(2,(r-1)) = -1;
                X_i(4,(r-1)) = -1;
                X_i(6,(r-1)) = -1;
                F(2,(r-1)) = -1;
                F(4,(r-1)) = -1;
            elseif (ROBOTS(r).neighbor_ID(t+1)) == 101
                
                
            else% When there is at least one neighbor
                X_i(2,(r-1)) = X_f(1,(ROBOTS(r).neighbor_ID(t+1)));
                X_i(4,(r-1)) = X_f(3,(ROBOTS(r).neighbor_ID(t+1)));
                X_i(6,(r-1)) = X_f(5,(ROBOTS(r).neighbor_ID(t+1)));
                F(2,(r-1)) = ROBOTS(ROBOTS(r).neighbor_ID(t+1)).velocity*cos(ROBOTS(ROBOTS(r).neighbor_ID(t+1)).pose(3)) + normrnd(0,.1,1,1);
                F(4,(r-1)) = ROBOTS(ROBOTS(r).neighbor_ID(t+1)).velocity*sin(ROBOTS(ROBOTS(r).neighbor_ID(t+1)).pose(3)) + normrnd(0,.1,1,1);
            end
        end
    end
    % PLOTEAR LOCATION FINAL
    quiver(X_i(1,:), X_i(pairwise+1,:),F(1,:), F(pairwise+1,:),0, 'b');
    % Plot vector with cartesian coordinates in each direction with 0 scale
    % (blue unit vectors)
    % X values, Y values, X increment vector, Y increment vector, scale
    plot(X_i(1,:), X_i(pairwise+1,:),'bo','MarkerSize',3)
    % Plot the points after each time step
    pause(1); % Pause 2 seconds for visualization purposes
end
hold off; % Stops holding the figure