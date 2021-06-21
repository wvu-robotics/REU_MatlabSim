
%% -------------------------- EKF SIMULATOR -------------------------------

number_of_robots = 100; % Number of Robots
dt = 1; % Increment of Time 1 second
ROBOTS = []; % Define the struct array for all the robots
range_of_robot = 10; % Radious of the laser
amount_of_steps = 10; % Define the amount of steps each robot takes
pairwise = 2; % Limit the communication between robots each time step

%% ------------------------- INITIALIZE ROBOTS ----------------------------

for r = 1:number_of_robots % For each robot in the total number of ROBOTS
    x = 10*rand(1,1); % Define array [1,1] value from 0 to 10
    y = 10*rand(1,1); % Define array [1,1] value from 0 to 10
    theta = 2*pi*rand(1,1); % Define array [1,1] value from 0 to 2*PI
    speed = 5*rand(1,1); % Define array [1,1] value from 0 to 5
    robot = struct('pose', [x,y,theta], 'velocity', speed, 'path', [x,y,theta], 'neighbor_ID', zeros(amount_of_steps,1), 'laser', zeros(amount_of_steps,1));
    % Asign values for different varaibles in the struct array 
    ROBOTS = [ROBOTS,robot]; % Combined struct array with variables robots
end

%% ------------------------- SIMULATE ROBOTS ------------------------------

for t = 1:amount_of_steps % For each time (1 second) in the total time (10 seconds)
    for r = 1:number_of_robots % For each robot in the total number of ROBOTS
        ROBOTS(r).pose = ROBOTS(r).pose + [ROBOTS(r).velocity*cos(ROBOTS(r).pose(3))*dt, ROBOTS(r).velocity*sin(ROBOTS(r).pose(3))*dt,0];
        % Calculate the true new postition based on the last postion and 
        % the velocity, increment of time and angle of displacement
        ROBOTS(r).path = [ROBOTS(r).path; ROBOTS(r).pose];
        % Add the new true position to the path array
    end
    % Make all the robots move firts
    for r = 1:number_of_robots % For each robot in the total number of ROBOTS
        distance = []; % Create array for distances
        for R = 1:number_of_robots % For each robot in the total number of ROBOTS
            distance = [distance, norm(ROBOTS(R).pose(1:2) - ROBOTS(r).pose(1:2)) + normrnd(0,.01,1,1)]; 
            % Calculate the normalized distance from each robot to the rest
            % of the robots at each time step with small white gaussian
            % error measurement (mean=0 and stdv= 0.01)
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
        else % If there is at least 1 neighbor
           random = randi(length(possible_distance)); % Pick a random distance
           neighbor = possible_distance(random); % Extract the neighbor ID
           distance_neighbor = distance(neighbor); % Extract the distance corresponding to that neighbor ID
        end
        ROBOTS(r).neighbor_ID(t)=neighbor;
        % Append the neighbour ID
        ROBOTS(r).laser(t) = distance_neighbor;
        % Append the distance to that neighbor ID
    end
    % Calculate relative distances from each robot to the other robots,
    % select a valid neighbor (in range and not itself) and append the
    % ID and the distance to them for each time step
end

%% ------------------------- PLOT THRUTH DATA -----------------------------

figure() % Create new figure
axis equal % Set aspect ratio in both axis equal
for r = 1:number_of_robots % For each robot in the total number of ROBOTS
    quiver(ROBOTS(r).path(:,1),ROBOTS(r).path(:,2),ROBOTS(r).velocity*cos(ROBOTS(r).pose(3))*ones(11,1), ROBOTS(r).velocity*sin(ROBOTS(r).pose(3))*ones(11,1),0,'r');
    % Plot vector with cartesian coordinates and in each direction with 0
    % scale
    hold on; % Hold the figure
end

%% --------------------- RUN FILTER ON GENERATED DATA ---------------------

X_i = zeros(pairwise*3,1); % Initialize the current state matrix
F = zeros(pairwise*3,1); % Initialize the kinematic gain matrix
Y_m = []; % Initialize the current measurement of the state ~ LIDAR
ID = []; % Initialize the ID list to track the neighbors

% Simulate the first step (No previous knowledge - No EKF)
for r = 1:number_of_robots % For each robot in the total number of ROBOTS
    X_i(1,r) = ROBOTS(r).path(2,1);
    % Set current state matrix with x component for robot 1
    X_i(2,r) = ROBOTS(ROBOTS(r).neighbor_ID(1)).path(2,1);
    % Set current state matrix with x component for robot 2
    X_i(3,r) = ROBOTS(r).path(2,2);
    % Set current state matrix with y component for robot 1
    X_i(4,r) = ROBOTS(ROBOTS(r).neighbor_ID(1)).path(2,2);
    % Set current state matrix with y component for robot 2
    X_i(5,r) = ROBOTS(r).path(2,3);
    % Set current state matrix with theta component for robot 1
    X_i(6,r) = ROBOTS(ROBOTS(r).neighbor_ID(1)).path(2,3);
    % Set current state matrix with theta component for robot 2
    
    F(1,r) = ROBOTS(r).velocity*cos(ROBOTS(r).pose(3));
    % Set kinematic gain to estimate next step for x component for robot 1
    F(2,r) = ROBOTS(ROBOTS(r).neighbor_ID(1)).velocity*cos(ROBOTS(ROBOTS(1).neighbor_ID(1)).pose(3));
    % Set kinematic gain to estimate next step for x component for robot 2
    F(3,r) = ROBOTS(r).velocity*sin(ROBOTS(r).pose(3));
    % Set kinematic gain to estimate next step for y component for robto 1
    F(4,r) = ROBOTS(ROBOTS(r).neighbor_ID(1)).velocity*sin(ROBOTS(ROBOTS(1).neighbor_ID(1)).pose(3));
    % Set kinematic gain to estimate next step for y component for robot 2
    % Already set the kinematic gain to estimate next step for theta
    % component equal to 0 for robot 1 and robot 2
    % F(5,r) = 0; 
    % F(6,r) = 0; 
    Y_m = ROBOTS(r).laser(1);
    % Add the first laser measurement from each robot relative to 
    % its neighbor to the current measurement of the matrix
    ID = ROBOTS(r).neighbor_ID(1);
    % Add the first neighbor ID recorded from each robot to the IDs list
    % each robot has
end
% plot(X_i(1,:), X_i(pairwise+1,:),'*b')
quiver(X_i(1,:), X_i(pairwise+1,:),F(1,:), F(pairwise+1,:),0, 'b');
% Plot vector with cartesian coordinates and in each direction with 0 scale
hold on; % Hold the figure

P_i = cell(1,number_of_robots); % Set the current covariance matrices cell (size)
for r = 1:number_of_robots
    P_i{r} = 0.1*eye(pairwise*3); % Set the current covariance matrix for each robot
end
Q = 1e-4*P_i{1}; % Set the estimation noise covariance matrix
R = 10*eye(pairwise*2); % Set the measurement noise covariance matrix
% Simulate the rest of the steps (Using EKF)
for t = 2 : amount_of_steps % For time step 2 to the rest
    for r = 1:number_of_robots % For each robot in the total number of ROBOTS
        Y_m = ROBOTS(r).laser(t); % Append the distance to the random neighbor
        ID = ROBOTS(r).neighbor_ID(t); % Append the ID from the neighbor
        [X_f,P_f] = Decentral_swarm_EKF(X_i(:,r), P_i{r}, Y_m, F(:,r), dt, Q, R);
        % EKF estimation
        X_i(:,r) = X_f; % Set the new current state matrix equal to the last a posteriori estimation for the state matrix
        P_i{r} = P_f; % Set the new current covariance matrix equal to the last a posteriori estimation for the covariance matrix
        % Y_m = []; % Clear the first laser data set from current measurement matrix 
        % ID = []; % Clear the first neighbor ID recorded
        % Y_m = [Y_m, ROBOTS(r).laser(t,:)]; % Append the distance to the random neighbor
        % ID = [ID, ROBOTS(r).neighbor_ID(1)]; % Append the ID from the neighbor
    end
    % plot(X_i(1,:), X_i(pairwise+1,:),'*b')
    quiver(X_i(1,:), X_i(pairwise+1,:),F(1,:), F(pairwise+1,:),0, 'b');
    % Plot vector with cartesian coordinates and in each direction with 0 scale
    hold on; % Hold the figure
    pause(2); % Pause 2 seconds for visualization purposes

end
hold off; % Stops holding the figure