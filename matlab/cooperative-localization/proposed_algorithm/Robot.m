classdef Robot
    properties
        %% class properties
        ID              % [int] id of the robot
        home            % [X, Y] home location of the robot
        goal            % [X, Y] goal location for the robot

        found_goal      % 0 = not found my current goal
                        % 1 = found my current goal

        estimator       % 0 = dead_reckoning,
                        % 1 = covariance intersection,
                        % 2 = decentralized EKF
                        % 3 = centralized EKF
        
        % positioning------------------------------------------------
        %dead reckoning
        position_d      % [X, Y, Yaw]
        velocity_d      % [Vx, Vy]
        covariance_d    % [3x3 X, Y, Yaw] covariance matrix
        path_d          % [X, Y, Yaw;   path taken by robot
                        %  .., ..,..];
        
        %estimated localization
        position_e      % [X, Y, Yaw]
        velocity_e      % [Vx, Vy]
        covariance_e    % [3x3 X, Y, Yaw] covariance matrix
        path_e          % [X, Y, Yaw;   path taken by robot
                        % ..,..,..];
        
        %truth
        position_t      % [X, Y, Yaw]
        velocity_t      % [Vx, Vy]
        path_t          % [X, Y, Yaw;   path taken by robot
                        %  .., ..,..];
        
        % sensors / measurment ----------------------------------------
        dt              % time step size
        vel_m           % velocity magnitude from wheel encoder
        yaw_rate_m      % yaw rate from wheel encoder
        laser           % [dist, dist, ...] lidar distances to every robot
        bearing         % [phi, phi, ...] global angle to every robot
        neighbors       % [robot.ID, robot.ID, ...] array of local robot's IDs
        state_particles % 2x1 cell array top cell holds state measurement of this robot from other robots
        %                lower cell holds covariances of the measurements
        
        detection_range % [double] detection range of the robot
        sigmaVelocity   % [double] wheel velocity variance
        biasVelocity    % [double] wheel velocity bias
        sigmaYawRate    % [double] wheel yaw rate variance
        biasYawRate     % [double] wheel yaw rate bias
        sigmaRange      % [double] lidar distance variance
        sigmaHeading    % [double] lidar angle variance
        
        %Boids parameters -------------------------------------------------
        
        acceleration    % [Ax, Ay] tell robot how to change its velocity
        max_force       % [double] limits maximum acceleration
        max_speed       % [double] velocity limit on the robot
        min_speed       % [double] minimum velocity command to move the robot
        max_yaw_rate    % [double] yaw rate limit on the robot
        Ks              % [double] seperation gain
        Ka              % [double] alignment gain
        Kc              % [double] cohesion gain
        Kh              % [double] home gain
        Kg              % [double] goal gain
        Kv              % [double] velocity gain
       
        % filtering variables --------------------------------------------
        X               % cell{ [X1; Y1; Yaw1], [X2,Y2,Yaw2], ...}
                        % stores the estimated states of all the robots
        
        P               % cell{ [3x3 1.1], [3x3 1.2], [3x3] 1.3, ... ;
                        %       [3x3 2.1], [3x3 2.2], [3x3] 2.3, ... ;
                        %       [3x3 3.1], [3x3 3.2], [3x3] 3.3, ...;
                        %        ...     ,   ...    ,    ...   , ... }
                        % stores the [X, Y, Yaw] covariance matrix for each
                        % robot and the cross covariances in the off
                        % diagonals
        
        % assimilation colors ----------------------------------------------
        color_particles
        
        % beacon parameters----------------------------------------------
        is_beacon
        time_as_beacon
        max_beacon_time
        
    end
    methods
        
        function obj = Robot(position_x,  position_y, Ks, Ka,Kc, numBoids, ID)
            obj.ID = ID;
            obj.found_goal = 1;
            
            % give initialize position and velocity
            angle = (2*pi).*rand;
            obj.velocity_d = [cos(angle), sin(angle)];
            obj.velocity_t = obj.velocity_d;
            obj.velocity_e = obj.velocity_d;
            
            obj.position_d = [position_x, position_y, angle];
            obj.position_t = obj.position_d;
            obj.position_e = obj.position_d;
            
            obj.path_d = [position_x, position_y, angle];
            obj.path_t =  obj.path_d;
            obj.path_e = obj.path_d;
            
            % initalize boids parameters-------------------------------------------
            obj.max_speed = 1;          % [m/s]
            obj.min_speed = .1;         % [m/s]
            obj.max_yaw_rate = .5;      % [rad/s]
            obj.max_force = 5;%0.1;
            obj.acceleration = [0 0];
            
            obj.Ks = Ks;
            obj.Ka = Ka;
            obj.Kc = Kc;
            obj.Kh = 3.23435;
            obj.Kg = 9.923;
            obj.Kv = 1;
            
            % initalize measurments ---------------------------------------
            
            obj.laser = zeros(1,numBoids);
            obj.bearing = obj.laser;
            obj.vel_m = norm(obj.velocity_t);
            obj.yaw_rate_m = 0;
            obj.state_particles = cell(2,1);
            
            
            % decentralized EKF parameters
            obj.X = cell (1,numBoids);
            obj.X{obj.ID} = obj.position_e';
            
            obj.P = cell(numBoids, numBoids);
            for i = 1:numBoids % For each robot
                for j = 1:numBoids % For each robot
                    if i == j
                        obj.P{i,j} = eye(3); % Covariance matrix
                    else
                        obj.P{i,j} = zeros(3); % Correlated matrix
                    end
                end
            end
            % color particle initalization
            obj.color_particles = zeros(1,3);
            
            % beacon initialization
            obj.time_as_beacon = 0;
            obj.is_beacon = 0; 
            obj.max_beacon_time = 20;
            
            
        end
        
        %% BOIDS FUNCTIONS ------------------------------------------------
        
        function obj = boids_update(obj,e_max,rho_max, cov_max,Lw)
           
            % measure local density
             A = pi*obj.detection_range^2;
             rho = length(obj.neighbors) / A;
             cov_e = norm(obj.covariance_e);
             dr = obj.detection_range;
             dh = norm(obj.position_e(1:2) - obj.home);
             dg = norm(obj.position_e(1:2) - obj.goal);
             e_m = norm(obj.position_e(1:2) - obj.position_d(1:2));
             V_max = obj.max_speed;
             V = obj.vel_m;
             tb = obj.time_as_beacon;
             t_max = obj.max_beacon_time;
             
             obj.Ka = (rho/rho_max)^2 + dr/dh ...
                      + relu(dlarray((e_m-e_max))).extractdata/e_max; 
             obj.Kc = cov_e/cov_max + (e_m/e_max)^2 ... 
                      + sigmoid(dlarray(dg/dr)).extractdata * dh/Lw;
             obj.Ks = relu(dlarray((cov_e-cov_max))).extractdata/cov_max...
                      + e_max/(e_m+1) + sigmoid(dlarray(dr/dh)).extractdata ...
                      +dg/Lw;
             obj.Kh = cov_e/cov_max + (e_m/e_max)^2 ...
                      + sigmoid(dlarray(dg/dr)).extractdata*((Lw-dh)/Lw);
             obj.Kg = (e_max/(e_m+1))^2 + (cov_max -cov_e)/cov_max ...
                      +sigmoid(dlarray(dh/dr)).extractdata*dr/dg;
            
%              temp = ((cov_max-cov_e)/cov_max)*((e_max-e_m)/e_max)...
%                     + obj.is_beacon*(tb-t_max)/t_max;
%              obj.Kv = relu(dlarray(temp)).extractdata;
             
             
             
            % old rules
            
%             covar = obj.covariance_e;
%             
%             mean_error = norm(obj.position_d(1:2) - obj.position_e(1:2));
%             % update boids parameters
%             obj.max_speed = (norm(covar)+2)/(norm(obj.position_e(1:2)-obj.position_e(1:2))+1)^2;
%             if obj.max_speed > 5
%                 obj.max_speed = 5;
%             end
%             obj.Ka = rho/rho_max + mean_error/e_max;
%             obj.Kc = (norm(covar) + mean_error^2)/A; %norm([norm(robot.covariance), mean_error^2]);
%             obj.Ks = A/(norm(covar) + mean_error^2);
%             obj.Kh = (mean_error^2 + norm(covar))/(norm(obj.home-obj.position_e(1:2))^2);
%             obj.Kg = obj.detection_range/(norm(obj.goal-obj.position_e(1:2))*norm(covar));
                
        end
        
        function obj = apply_force(obj,sep_force,coh_force,ali_force)

            home_force = obj.seek(obj.home);
            if obj.found_goal == 1
                temp = 0;
            else
                temp = obj.Kg;
            end
            goal_force = obj.seek(obj.goal);
            force = sep_force+coh_force+ali_force+obj.Kh*home_force+ temp*goal_force;
            if norm(force) ~= 0
                obj.acceleration = obj.max_force * force / norm(force);
            else
                obj.acceleration = [0,0];
            end
            
        end

        function obj = flock(obj,boids)
            sep = obj.seperate(boids);
            ali = obj.align(boids);
            coh = obj.cohesion(boids);
            
            sep = sep.*obj.Ks;%15;
            ali = ali.*obj.Ka;%1.0;
            coh = coh.*obj.Kc;%1.0;
            
            obj=obj.apply_force(sep,coh,ali);
        end
        
        function [steer] = seek(obj,target)
            desired = target - obj.position_e(1:2);
            desired = desired/norm(desired);
            desired = desired*obj.max_speed;
            
            steer = desired-obj.velocity_e;
            steer = steer.*obj.max_force/norm(steer);
        end
        
        function [steer] = seperate(obj,boids)
            desired_separation = obj.detection_range; %%%%%%%%%% communication range
            steer = [0,0];
            count = 0;
            positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                positions(:,i) = boids(i).position_e(1:2);
            end
            d = pdist([obj.position_e(1:2); positions']);
            d = d(1:length(boids));
            
            for i=1:1:length(boids)
                if d(i) > 0 && d(i) <  desired_separation
                    difference = obj.position_e(1:2) - boids(i).position_e(1:2);
                    difference = difference./norm(difference);
                    difference = difference./d(i);
                    steer = steer + difference;
                    count = count+1;
                end
                
                if count > 0
                    steer = steer./count;
                end
                
                if norm(steer) > 0
                    steer = steer./norm(steer).*obj.max_speed;
                    steer = steer - obj.velocity_e;
                    steer = steer./norm(steer).*obj.max_force;
                end
            end
        end
        
        function steer = align(obj,boids)
            neighbor_dist = obj.detection_range;
            sum = [0 0];
            count = 0;
            steer = [0 0];
            
            positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                positions(:,i) = boids(i).position_e(1:2);
            end
            d = pdist([obj.position_e(1:2); positions']);
            d = d(1:length(boids));
            
            for i=1:1:length(boids)
                if d(i)>0 && d(i) < neighbor_dist
                    sum=sum+boids(i).position_e(1:2);
                    count=count+1;
                end
            end
            
            if count > 0
                sum=sum./count;
                sum=sum./norm(sum).*obj.max_speed;
                steer=sum-obj.velocity_e;
                steer=steer./norm(steer).*obj.max_force;
            end
        end
        
        function steer = cohesion(obj,boids)
            neighbor_dist = obj.detection_range;%50;
            sum = [0 0];
            count = 0;
            steer = [0 0];
            
            positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                positions(:,i) = boids(i).position_e(1:2);
            end
            d = pdist([obj.position_e(1:2); positions']);
            d = d(1:length(boids));
            
            for i=1:1:length(boids)
                if d(i)>0 && d(i) < neighbor_dist
                    sum=sum+boids(i).position_e(1:2);
                    count=count+1;
                end
            end
            
            if count > 0
                sum=sum./count;
                steer = obj.seek(sum);
            end
        end
        
        %% MEASUREMENTS ---------------------------------------------------
        
        function obj = lidar_measurement(obj,ROBOTS)
            
            dists = []; %distances to every robot
            angles = [];%global angle to every robot
            numBots = length(ROBOTS);
            r = obj.ID;  % this robot
            neigh = [];  % neighbor robot ID's
            
            for L = 1:numBots % other robot
                
                %distance from this robot (r) to other robot (L)
                d = norm(ROBOTS(L).position_t(1:2)- ROBOTS(r).position_t(1:2)); %truth
                d = d + normrnd(0,ROBOTS(r).sigmaRange,1,1); %noise
                %angle from r to L
                phi = atan2(ROBOTS(L).position_t(2)- ROBOTS(r).position_t(2), ROBOTS(L).position_t(1)- ROBOTS(r).position_t(1)); % truth
                phi = phi + normrnd(0,ROBOTS(r).sigmaHeading,1,1); %noise
                phi = phi + angdiff(ROBOTS(r).position_e(3), ROBOTS(r).position_t(3)); %bias
                
                dists = [dists, d];
                angles = [angles, phi];

                detect_rng = obj.detection_range;
                if obj.estimator == 3
                    detect_rng = 50;
                end
                if d < detect_rng && r ~= L
                    neigh = [neigh, ROBOTS(L).ID];
                end
            end
            
            obj.laser = dists;
            obj.bearing = angles;
            obj.neighbors = neigh;
            
        end
        
        function obj = encoder_measurement(obj)
            % get true velocity
            [V_t,yaw_rate_t] = obj.Ag2V();
            
            %  measure velocity
            obj.vel_m = V_t; %truth
            obj.vel_m = obj.vel_m+ normrnd(0,obj.sigmaVelocity,1,1); %noise 
            obj.vel_m = obj.vel_m+ obj.biasVelocity; % bias
           
            % measure yaw
            obj.yaw_rate_m = yaw_rate_t; %truth
            obj.yaw_rate_m = obj.yaw_rate_m + normrnd(0,obj.sigmaYawRate,1,1); % noise
            obj.yaw_rate_m = obj.yaw_rate_m + obj.biasYawRate; % bias
            
        end

         function obj = home_update(obj,home_range)
            
            home_dist = norm(obj.position_t(1:2) - obj.home);
            if home_dist < home_range
                % update location for dead and estimated to truth at t+1
                
                obj.covariance_d = [.1,  0,  0;
                                     0, .1   0;
                                     0,  0, .1];
                obj.covariance_e = obj.covariance_d;
                
                obj.position_d = obj.position_t + normrnd(0,.001,1,3);
                obj.position_e = obj.position_d;
               % obj.position_e(3) = mod(obj.position_e(3),2*pi);
                
                %% get color particles from home
                if home_dist < home_range %within 5 squares
                    theta = atan2d(obj.position_t(2) - obj.home(2),obj.position_t(1) - obj.home(1));
                    if theta < -120 %red range
                        obj.color_particles(1) = obj.color_particles(1) + 5;
                    elseif theta < 0 %green
                        obj.color_particles(2) = obj.color_particles(2) + 5;
                    elseif theta < 120 %blue
                        obj.color_particles(3) = obj.color_particles(3) + 5;
                    else % red range
                        obj.color_particles(1) = obj.color_particles(1) + 5;
                    end
                    
                end
            end
        end
        
        function [X,P] = get_states(obj,ROBOTS)
            numBots = length(ROBOTS);
            X = obj.X;
            P = obj.P;
            for i = 1:numBots
                X{i} = ROBOTS(i).position_e';
                for j = 1:numBots
                    if i == j
                        P{i,i} = ROBOTS(i).covariance_e;
                    else
                        P{i,j} = ROBOTS(i).P{i,j};
                    end
                    
                end
            end
        end
        
        function [states, covars] = get_locations(obj,ROBOTS)
            
            numbots = length(ROBOTS);
            states = obj.X{obj.ID};
            covars = obj.P{obj.ID,obj.ID};
            for L = 1:numbots % other robot
                d1_2 = ROBOTS(L).laser(obj.ID);
                d2_1 = obj.laser(L);
                phi2_1 = obj.bearing(L);
                phi1_2 = ROBOTS(L).bearing(obj.ID);
                
                detect_rng = obj.detection_range;
                if obj.estimator == 3
                    detect_rng = 50;
                end

                if L ~= obj.ID && d1_2 < detect_rng
                    
                    x1_2 = obj.X{L}(1) + d1_2 * cos(phi1_2);
                    y1_2 = obj.X{L}(2) + d1_2 * sin(phi1_2);
                    yaw1_2 = obj.X{obj.ID}(3) - angdiff(phi2_1,phi1_2+pi);
                    

                    R = diag([obj.sigmaRange, obj.sigmaRange,obj.sigmaHeading]);
                    
                    states(:,end+1) = [x1_2;y1_2;yaw1_2];
                    covars(:,:,end+1) = obj.P{L,L} + R;
                    
                end
%%                 d = obj.laser(L);
%                 phi = obj.bearing(L)+pi;
%                 
%                 %if the other robot is in detection / communication range
%                 %give it our dead-reckoning prediction of where it is
%                 %update our particle if we have one there already
%                 
%                 if L ~= obj.ID
%                     dx = d*cos(phi); %from r -> L
%                     dy = d*sin(phi); %from r -> L
%                     x0 = obj.X{L}(1);%ROBOTS(L).position_e(1); %r pose
%                     y0 = obj.X{L}(2);%ROBOTS(L).position_e(2); %r pose
%                     theta = obj.X{obj.ID}(3);%ROBOTS(L).position_e(3);
%                     %give L, r's position of L, and mark that it has that
%                     %particle
%                     states(:,end+1) = [x0+dx;y0+dy];
%                     covars(:,:,end+1) = ROBOTS(L).covariance_d(1:2,1:2);%obj.P{L,L}(1:2,1:2);
%                     
%                 end
                
            end
            %covars = covars(:,:,2:end);
            
        end
        
        %% KINEMATIC UPDATE -----------------------------------------------
        
        function obj = update(obj)
            
            %determine if we are a beacon-------------------------------------------
            if obj.is_beacon == 1 && rand > obj.time_as_beacon/100 % remain a beacon
                obj.velocity_e = [0,0];
                obj.velocity_t = [0,0];
                obj.velocity_d = [0,0];
            elseif obj.is_beacon == 1 % stop becoming a beacon
                obj.is_beacon = 0;
            else                     % i am not a beacon
                
                
                %perform dead_reckoning
                obj = obj.dead_reckoning(); % set dead reckoning position to t+1
                
                % update estimate of location
                obj = obj.estimate_location(); % set estimated location to t + 1
                
                
                % update truth velocity and position to t+1
                
                [V,yaw_rate] = obj.Ag2V();    % get NEW velocity
                new_theta = obj.position_t(3) + yaw_rate*obj.dt;  
                obj.velocity_t = V*[cos(new_theta), sin(new_theta)];
                    % true position at t + 1
                obj.position_t = [obj.position_t(1:2) + obj.velocity_t*obj.dt, new_theta]; 
                
                % Home Measurement
                obj = obj.home_update(obj.detection_range);
                
                %record paths
                obj.path_t = [obj.path_t; obj.position_t];
                obj.path_d = [obj.path_d; obj.position_d];
                obj.path_e = [obj.path_e; obj.position_e];
                
                %set acceleration to zero
                obj.acceleration = [0 0];
                
                % check if we reached a goal or not
                if norm(obj.position_e(1:2) - obj.goal) < obj.detection_range
                    obj.found_goal = 1;
                end
                
            end
        end
        
        function [V,yaw_rate] = Ag2V(obj)
            %covert Accelleration to velocity and yaw_rate
            ax = obj.acceleration(1);

            ay = obj.acceleration(2);
            
            theta = obj.position_t(3); % global position
            V = obj.Kv*(norm(obj.velocity_t) + (ax*cos(-theta) - ay*sin(-theta))*obj.dt);
            
            
            if V > obj.max_speed    % bound the velocity
                V = obj.max_speed;
            elseif V < obj.min_speed
                V = obj.min_speed;
            end
            
            % calculate yaw error between acceleration vector and current
            % heading
            if norm(obj.acceleration) ~= 0
                desired = obj.acceleration/norm(obj.acceleration);
                desired_yaw = atan2(desired(2),desired(1));
                yaw_error = -angdiff(desired_yaw,obj.position_t(3));
                yaw_rate = obj.Kv*yaw_error;
                
                if yaw_rate > obj.max_yaw_rate     % bound the yaw rate
                    yaw_rate = obj.max_yaw_rate;
                elseif yaw_rate < -obj.max_yaw_rate 
                    yaw_rate = -obj.max_yaw_rate;
                end
            else
                yaw_rate = 0; % if no acceleration yaw_rate = 0;
            end
            
        end
        
        %% LOCALIZATION FUNCTIONS -----------------------------------------
        
        function obj = estimate_location(obj)
            switch (obj.estimator)
                case 0 % just use dead_reckoning
                    %dead reckoning already occured so it is already at t+1
                    obj.position_e = obj.position_d; 
                    obj.velocity_e = obj.velocity_d;
                    obj.covariance_e = obj.covariance_d;
                case 1 % covariance intersection
                    obj = obj.covariance_intersection();
                case 2 % decentralized ekf
                    obj = obj.Decentralized_EKF();
                case 3 % centralized ekf
                    %obj = obj.Centralized_EKF();
                    obj = obj.Decentralized_EKF();
            end
        end
        
        function obj = dead_reckoning(obj)
            % update dead reckoning
            new_theta = obj.position_d(3) + obj.yaw_rate_m*obj.dt;
            obj.velocity_d = obj.vel_m*[cos(new_theta), sin(new_theta)];
            obj.position_d = [obj.position_d(1:2) + obj.velocity_d*obj.dt, new_theta];
            
            F_d = [1,0,-obj.vel_m*sin(new_theta)*obj.dt;  % X
                   0,1, obj.vel_m*cos(new_theta)*obj.dt;  % Y
                   0,0,           1             ]; % Yaw
            
            
            Q_d = diag([obj.sigmaVelocity, obj.sigmaVelocity, obj.sigmaYawRate]);
            
            old_covar = obj.covariance_d;
            obj.covariance_d = F_d*obj.covariance_d*(F_d') + Q_d;
           
            
        end
       
        
        % COVARIANCE INTERSECTION -----------------------------------------
        
        function obj = covariance_intersection(obj)
            % CURRENT states and covariances estimates from other robots
            states = obj.state_particles{1};  
            covars = obj.state_particles{2};
            
            %------------------------------ CI -----------------------
            if length(obj.neighbors) > 1
               
                
                %---------------------------FAST CI
                %[mean_pose,covar] = fusecovint(states,covars);
                [mean_pose,covar] = fast_CI(states,covars);
                
                %---------------------------------------------

                % lidar measurement noise
               % R = diag([obj.sigmaRange, obj.sigmaRange,obj.sigmaHeading]);
                R = diag([0, 0, 0]);
                % fused estimate of the robot's CURRENT STATE
                obj.position_e = mean_pose';
                obj.covariance_e = covar + R;
               
                
                %%  NOT USED CURRENTLY ---------------------use CI with Kalman update --------------
            
%                 X_p = obj.position_e';
%                 P_p = obj.covariance_e;
%                 Y_p = obj.position_e(1:2)';
% 
%                 Y_m = mean_pose;
%                 P_m = covar;
% 
%                 H = [1,0,0;
%                      0,1,0];
% 
%                 I = eye(3);
% 
%                 residual = Y_m - Y_p;
%                 S = H*P_p*H' + P_m;
%                 K = P_p*H'*inv(S);
%                 obj.position_e = (X_p + K*residual)';
%                 obj.covariance_e = (I-K*H)*P_p;
                
            end
            
            %-------------------------dead reckoning update---------------
            new_theta = obj.position_e(3) + obj.yaw_rate_m*obj.dt;
            obj.velocity_e = obj.vel_m*[cos(new_theta), sin(new_theta)];
            %new_theta = mod(new_theta,2*pi);
            obj.position_e = [obj.position_e(1:2) + obj.velocity_e*obj.dt, new_theta]; % update estimated position to t+1
            
            F_d = [1,0,-obj.vel_m*sin(new_theta)*obj.dt;     % X
                   0,1, obj.vel_m*cos(new_theta)*obj.dt;     % Y
                   0,0,           1             ];           % Yaw
            
            Q_d = diag([obj.sigmaVelocity, obj.sigmaVelocity, obj.sigmaYawRate]);
            
            old_covar = obj.covariance_e; % from CI
            
            obj.covariance_e = F_d*obj.covariance_e*F_d' + Q_d;
            
            %------------------------------------------------------
            
        end

        % DECENTRALIZED ---------------------------------------------------

        function obj= Decentralized_EKF(obj)
            [obj] = Relative_Measurement_Update(obj);
            %-------------------------dead reckoning update---------------
            new_theta = obj.position_e(3) + obj.yaw_rate_m*obj.dt;
            obj.velocity_e = obj.vel_m*[cos(new_theta), sin(new_theta)];
            %new_theta = mod(new_theta,2*pi);
            obj.position_e = [obj.position_e(1:2) + obj.velocity_e*obj.dt, new_theta];
            
            F_d = [1,0,-obj.vel_m*sin(new_theta)*obj.dt;  % X
                   0,1, obj.vel_m*cos(new_theta)*obj.dt;     % Y
                   0,0,           1             ];           % Yaw
            
            Q_d = diag([obj.sigmaVelocity, obj.sigmaVelocity, obj.sigmaYawRate]);
            
            old_covar = obj.covariance_e;
            
            obj.covariance_e = F_d*obj.covariance_e*F_d' + Q_d;


            
            %------------------------------------------------------
            
            %if obj.neighbors == 0
            %    obj.neighbors = [];
            %end
        end
         %  CENTRALIZED EKF ----------------------------------------------         
        function obj= Centralized_EKF(obj)
            [obj] = Relative_Measurement_Update(obj);
            %-------------------------dead reckoning update---------------
            new_theta = obj.position_e(3) + obj.yaw_rate_m*obj.dt;
            obj.velocity_e = obj.vel_m*[cos(new_theta), sin(new_theta)];
            %new_theta = mod(new_theta,2*pi);
            obj.position_e = [obj.position_e(1:2) + obj.velocity_e*obj.dt, new_theta];
            
            F_d = [1,0,-obj.vel_m*sin(new_theta)*obj.dt;  % X
                0,1, obj.vel_m*cos(new_theta)*obj.dt;     % Y
                0,0,           1             ];           % Yaw
            
            Q_d = diag([obj.sigmaVelocity, obj.sigmaVelocity, obj.sigmaYawRate]);
            
            old_covar = obj.covariance_e;
            
            obj.covariance_e = F_d*obj.covariance_e*F_d' + Q_d;
            
            %------------------------------------------------------
            
        end
       
        
        function obj = Relative_Measurement_Update(obj)

            %Parameters
            i = obj.ID; % Define robot i
            for k = 1:size(obj.neighbors,2) % For each neighbor
                j = obj.neighbors(k); % Being j the neighbor for robot i
                if j == 0 % No neighbor
                    % Do nothing
                else
                    % This funcion runs a relative measurement update for robot i
                    Yr = [obj.laser(j);obj.bearing(j)]; % Measurement for robot i to robot j
                    Xi = obj.X{i}; % State robot i
                    Xj = obj.X{j}; % State robot j
                    Pi = obj.P(i,:); % Covariance and correlated values for robot i
                    Pj = obj.P(j,:); % Covariance and correlated values for robot j
                    number_of_robots = size(Pi,2); % Set the number of robots
                    number_of_robots = size(Pj,2); % Set the number of robots

                    Xx = [Xi ; Xj]; % Combined State matrix
                    Pi_i = Pi{1,i}; % Set the covariance matrix for robot i
                    Pj_i = Pj{1,j}; % Set the covariance matrix for robot j
                    Ci_i = Pi{1,j}; % Correlated value from robot i to robot j
                    Cj_i = Pj{1,i}; % Correlated value from robot j to robot i
                    Pij = Ci_i * Cj_i'; % Correlated values between robot i and robot j
                    Px = [Pi_i Pij ; Pij' Pj_i]; % Combined covariance matrix
                    I = eye(6); % Identity Matrix
                    %%% ASSUMPTIONS %%%
                    % If robot i detects robot j, robot j detects robot i -> (360º detection)
                    % R is the measurement noise covariance
                    % White Gaussian noise N(0,r) with 0 mean and r stdv
                    R = obj.sigmaRange * eye(2); % Estimation noise covariance
                    %%% INPUT %%%
                    % Xi is the current state matrix for robot i
                    % Pi_i is the current covariance matrix for robot i
                    % Ci_i is the current correlated matrixes for robot i to the rest
                    % Pi is the combination of Pi_i and Ci_i
                    % Xj is the current state matrix for robot j
                    % Pj_i is the current covariance matrix for robot j
                    % Cj_i is the current correlated matrixes for robot j to the rest
                    % Pj is the combination of Pj_i and Cj_i
                    % i refers to the number of the robot i
                    % j refers to the number of the robot j
                    % Y is the relative measurement from robot i to j / robot j to i
                    % Yr is the measurement with noise
                    rho = Yr(1); % rho is the distance from robot i to rotob j with noise in the measurement
                    phi = Yr(2); % phi is the angle from robot i to robot j with noise in the measurement
                    %%% OUTPUT %%%
                    % Xip is the a priori estimation for the state matrix for robot i
                    % Pi_p is the a priori estimation for the covariance matrix for robot i
                    % Ci_p is the a priori estimation for the correlated matrixes for robot i to the rest
                    % Pip is the combination of Pi_p and Ci_p
                    % Xjp is the a priori estimation for the state matrix for robot j
                    % Pj_p is the a priori estimation for the covariance matrix for robot j
                    % Cj_p is the a priori estimation for the correlated matrixes for robot j to the rest
                    % Pjp is the combination of Pj_p and Cj_p
                    
                    % Expected Measurement
                    h = zeros(2,1); % Initialize
                    h(1,1) = sqrt(((Xj(1)-Xi(1))^2)+((Xj(2)-Xi(2))^2)); % Expected rho measurement
                    h(2,1) = atan2(Xj(2)-Xi(2),Xj(1)-Xi(1)); % Expected phi measurement
                    if -1 < Xj(2)-Xi(2) && Xj(2)-Xi(2) < 1
                        difference_bearing_1 = abs(Yr(2)-atan2(Xj(2)-Xi(2),Xj(1)-Xi(1))); % Normal Sign;
                        difference_bearing_2 = abs(Yr(2)+atan2(Xj(2)-Xi(2),Xj(1)-Xi(1))); % Change sign;
                        if difference_bearing_1 < difference_bearing_2 % If correct
                            h(2,1) = atan2(Xj(2)-Xi(2),Xj(1)-Xi(1)); % Expected phi measurement
                        else % If you need to change sign
                            h(2,1) = -atan2(Xj(2)-Xi(2),Xj(1)-Xi(1)); % Expected phi measurement
                        end
                    end
            
                    % Linearize
                    H(1,:) = [ -(Xj(1)-Xi(1))/(sqrt(((Xj(1)-Xi(1))^2)+((Xj(2)-Xi(2))^2))) , -(Xj(2)-Xi(2))/(sqrt(((Xj(1)-Xi(1))^2)+((Xj(2)-Xi(2))^2))) , 0 , (Xj(1)-Xi(1))/(sqrt(((Xj(1)-Xi(1))^2)+((Xj(2)-Xi(2))^2))) , (Xj(2)-Xi(2))/(sqrt(((Xj(1)-Xi(1))^2)+((Xj(2)-Xi(2))^2))) , 0]; % Jacobbian for rho input
                    H(2,:) = [ (Xj(2)-Xi(2))/(((Xj(1)-Xi(1))^2)+((Xj(2)-Xi(2))^2)) , -(Xj(1)-Xi(1))/(((Xj(1)-Xi(1))^2)+((Xj(2)-Xi(2))^2)) , 0 , -(Xj(2)-Xi(2))/(((Xj(1)-Xi(1))^2)+((Xj(2)-Xi(2))^2)) , (Xj(1)-Xi(1))/(((Xj(1)-Xi(1))^2)+((Xj(2)-Xi(2))^2)) , 0]; % Jacobbian for phi input


                    S = H * Px * H' + R; % Innovation Covariance Formula
                    K = Px * H' * inv(S); % Kalman Gain Formula

                    RESIDUAL = Yr - h; % Residual Formula 

                    Xp = Xx + K * RESIDUAL; % A posteriori estimation for the combined state matrix
                    Xip = Xp(1:3); % A posteriori estimation for the state matrix of robot i
                    Xjp = Xp(4:6); % A posteriori estimation for the state matrix of robot j 

                    
                    hey = abs(Xp - obj.position_t);
                    
                    if hey > 1
                        disp('---------------------------------------------------------------')
                        disp(i)
                        disp('True i')
                        disp(obj.position_t)
                        disp('A priori Estimation i')
                        disp(Xi)
                        disp('A posteriori Estimation i')
                        disp(Xip)
                        disp('---------------------------------------------------------------') 
                    end
                    
                    %%% --------------------- CHECK WRONG MEASUREMENTS -------------------- %%%
                    %%% ------------------------------------------------------------------- %%%
                    error_x_i = Xi(1) - Xip(1);
                    error_y_i = Xi(2) - Xip(2);
                    error_x_j = Xj(1) - Xjp(1);
                    error_y_j = Xj(2) - Xjp(2);
                    check = 1; % Set limit to display message cause there might be a wrong update
                    if error_x_i > check || error_y_i > check || error_x_j > check || error_y_j > check
                        disp('---------------------------------------------------------------')
                        disp(i)
                        disp('True i')
                        disp(obj.position_t)
                        disp('A priori Estimation i')
                        disp(Xi)
                        disp('A posteriori Estimation i')
                        disp(Xip)
                        disp('---------------------------------------------------------------')
                        disp(j)
                        disp('A priori Estimation j')
                        disp(Xj)
                        disp('A posteriori Estimation j')
                        disp(Xjp)
                        disp('---------------------------------------------------------------')
                        disp('Measurement')
                        disp(Yr)
                        disp('Estimation')
                        disp(h')
                        disp('Checked')
                        disp('---------------------------------------------------------------')
                    end
                    %%% ------------------------------------------------------------------- %%%
                         
                    Pp = (I - K * H) * Px; % A posteriori estimation for the combined covariance matrix

                    Pip = cell(1,number_of_robots); % Initialize
                    Pip{1,i} = Pp(1:3,1:3); % Covariance a for robot i
                    Pip{1,j} = Pp(1:3,4:6); % Correlated value from robot i to robot j
                    for k = 1:number_of_robots % For each robot
                        if k == i
                            % Do nothing
                        elseif k == j
                            % Do nothing
                        else
                            Pip{1,k} = Pip{1,i} * inv(Pi{1,i}) * Pi{1,k}; % Correlated values from robot i to the rest of the robots
                        end
                    end

                    Pjp = cell(1,number_of_robots); % Initialize
                    Pjp{1,i} = eye(3); % Correlated values for robot j to robot i
                    Pjp{1,j} = Pp(4:6,4:6); % Covariance for robot j
                    for k= 1:number_of_robots
                        if k==i
                            % Do nothing
                        elseif i ==j
                            % Do nothing
                        else
                            Pjp{1,k} = Pjp{1,j} * inv(Pj{1,j}) * Pj{1,k};  % Correlated values from robot j to the rest of the robots
                        end
                    end
                    
                    
                    
                    Error = sqrt(((Xip(1,1)-obj.X{i}(1,1))^2)+((Xip(2,1)-obj.X{i}(2,1))^2)); % Error
                    if Error < .5
                        obj.covariance_e = Pip{1,i}; % Update Covariance and correlated values for robot i            
                        obj.position_e = Xip'; % Update location robot i
                        obj.X{i} = Xip;
                        obj.P{i,:} = Pip;
                        obj.X{j} = Xjp; % Update location robot j
                        obj.P(j,:) = Pjp; % Update Covariance and correlated values for robot j
                    else
                        % Don´t Update
                        disp("EXCEEDED ERROR THRESHOLD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                        disp(Error);
                    end   
                end     
            end
        end
        
        %% BEACON FUNCTIONS -----------------------------------------------
        
        % NEEDS TO BE FIXED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        function ROBOTS = beacon_update(obj,ROBOTS,cov_max)
            
            found_beacon = 0;
            beacon = [];
            % if norm(ROBOTS(ID).mean_covar) < cov_max
            %     ROBOTS(ID).is_beacon =0;
            % end
            
            if ROBOTS(obj.ID).is_beacon == 1 && length(obj.neighbors) <= 1
                ROBOTS(obj.ID).time_as_beacon = ROBOTS(obj.ID).time_as_beacon+1;
                found_beacon = 1;
                beacon = obj.ID;
            end
            
            for r = obj.neighbors
                if ROBOTS(r).is_beacon == 1
                    found_beacon =1;
                    if length(obj.neighbors) <= 1
                        ROBOTS(r).time_as_beacon = ROBOTS(r).time_as_beacon+1;
                    else
                        ROBOTS(r).time_as_beacon = 0;
                    end
                    beacon = r;
                    break;
                end
            end
            
            if found_beacon == 1
                for r = obj.neighbors
                    if norm(ROBOTS(r).covariance_e) > norm(ROBOTS(beacon).covariance_e) %may need to flip the sign
                        ROBOTS(beacon).is_beacon = 0;
                        ROBOTS(beacon).time_as_beacon = 0;
                        ROBOTS(r).is_beacon = 1;
                        ROBOTS(r).time_as_beacon = 0;
                        beacon = r;
                    end
                end
            else
                if norm(ROBOTS(obj.ID).covariance_e) > cov_max
                    ROBOTS(obj.ID).is_beacon =1;
                    ROBOTS(obj.ID).time_as_beacon = 0;
                end
                
            end
  
        end
        
        %% COLOR PARTICLE FUNCTIONS ---------------------------------------
        
        function [obj, other] = trade_color(obj,other)
            %calculate probability weights
            W = other.color_particles./ sum(other.color_particles);
            
            %check to make sure the other agent has a
            %particle
            if sum(W) > 0
                color = randsample(1:3,1,true,W); %pick color particle
                obj.color_particles(color) = obj.color_particles(color)+1; %recieve particle
                other.color_particles(color) = other.color_particles(color)-1; %remove particle from other agent
            end
        end
        
        %% DISPLAY FUNCTIONS ----------------------------------------------
        
        function disp_swarm(ROBOTS,range,show_detection_rng)
            
            numBots = length(ROBOTS);
            KA = [];
            KC = [];
            KS = [];
            KH = [];
            KG = [];
            clf();
            %subplot(2,3,1);
            for r = 1:numBots
                % plot truth data-----------------------------------------
                
                plot(ROBOTS(r).path_t(end,1), ROBOTS(r).path_t(end,2), 'ks');
                hold on;
                quiver(ROBOTS(r).position_t(1), ROBOTS(r).position_t(2),ROBOTS(r).velocity_t(1), ROBOTS(r).velocity_t(2), 'b');
                hold on;
                if show_detection_rng
                    viscircles([ROBOTS(r).path_t(end,1),ROBOTS(r).path_t(end,2)],range);
                    hold on;
                end
                %plot estimated position and color-------------------------------------------
                error_ellipse(ROBOTS(r).covariance_e(1:2,1:2), [ROBOTS(r).position_e(1), ROBOTS(r).position_e(2)],'conf',.5)
                hold on;
                if sum(ROBOTS(r).color_particles) > 0
                    COLOR= ROBOTS(r).color_particles./sum(ROBOTS(r).color_particles);
                else
                    COLOR = [0,0,0];
                end
                
                if ROBOTS(r).is_beacon == 1
                    plot(ROBOTS(r).position_e(1), ROBOTS(r).position_e(2), '^', 'color', COLOR);
                    hold on;
                else
                    plot(ROBOTS(r).position_e(1), ROBOTS(r).position_e(2), '*', 'color', COLOR);
                    hold on;
                end
                % plot dead reckoning position---------------------------------------------------------
                plot(ROBOTS(r).position_d(1), ROBOTS(r).position_d(2), 'bo');
                hold on;
                
                
                %plot goal position---------------------------------------------
                plot(ROBOTS(r).goal(1), ROBOTS(r).goal(2), 'rx')
                
                KA = [KA,ROBOTS(r).Ka];
                KC = [KC,ROBOTS(r).Kc];
                KS = [KS,ROBOTS(r).Ks];
                KH = [KH,ROBOTS(r).Kh];
                KG = [KG,ROBOTS(r).Kg];
            end
            %plot home------------------------------------------------------
            plot(ROBOTS(1).home(1),ROBOTS(1).home(2), 'bd','markersize',12)
            hold on;
            viscircles([ROBOTS(1).home(1),ROBOTS(1).home(2)],range);
            hold on;
            title("Square = truth, * = estimate, o = dead reckoning");
            axis([-10 10 -10 10])
            
            %plot gain distributions
%                 subplot(2,3,2)
%                 histogram(KA)
%                 title("alignment gain")
%                 subplot(2,3,3)
%                 histogram(KC)
%                 title("cohesion gain")
%                 subplot(2,3,4)
%                 histogram(KS)
%                 title("Seperation gain")
%                 subplot(2,3,5)
%                 histogram(KH)
%                 title("Home gain")
%                 subplot(2,3,6)
%                 histogram(KG)
%                 title("Goal gain")
            
            
            pause(.0001);
        end
               
    end
end
