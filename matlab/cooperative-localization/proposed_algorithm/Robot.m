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
                        %  .., ..,..];
                        
        %truth
        position_t      % [X, Y, Yaw]
        velocity_t      % [Vx, Vy]
        path_t          % [X, Y, Yaw;   path taken by robot
                        %  .., ..,..];
        
        % sensors / measurment ----------------------------------------
        
        vel_m           % velocity magnitude from wheel encoder
        yaw_rate_m      % yaw rate from wheel encoder
        laser           % [dist, dist, ...] lidar distances to every robot
        bearing         % [phi, phi, ...] global angle to every robot
        neighbors       % [robot, robot, ...] array of local robots
        
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
        Ks              % [double] seperation gain
        Ka              % [double] alignment gain
        Kc              % [double] cohesion gain
        Kh              % [double] home gain
        Kg              % [double] goal gain
        
        % covariance intersection ----------------------------------------
        state_particles
        
        % decentralized EKF parameters -----------------------------------
        P
        X
        % assimilation colors ----------------------------------------------
        color_particles
        
        % beacon parameters----------------------------------------------
        is_beacon
        time_as_beacon
        
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
            obj.path_e = obj.path_d(1:2);
            
            % initalize boids parameters-------------------------------------------
            obj.max_speed = 5;
            obj.max_force = 0.1;
            obj.acceleration = [0 0];
            
            obj.Ks = Ks;
            obj.Ka = Ka;
            obj.Kc = Kc;
            obj.Kh = 1;
            obj.Kg = 0;
            
            % initalize measurments ---------------------------------------
            
            obj.laser = zeros(1,numBoids);
            obj.bearing = obj.laser;
            obj.vel_m = norm(obj.velocity_t);
            obj.yaw_rate_m = 0;
            
            % covariance intersection initalization
            obj.state_particles = cell(2,1);
            obj.state_particles{1} = zeros(3,numBoids);
            obj.state_particles{2} = zeros(2,2,numBoids);
            
            % decentralized EKF parameters
            obj.X = cell (1,numBoids);
            for i = 1:numbBoids % For each robot
                x = 10 * rand(1,1); % Set a random x value
                y = 10 * rand(1,1); % Set a random y value
                theta = 2 * pi * rand(1,1); % Set a random theta value
                X{i} = [x;y;theta]; % Combined state matrix
            end
            obj.P = cell(numBoids, numBoids);
            for i = 1:numBoids % For each robot
                P{i,i} = eye(3); % Covariance matrix
                for j = 1:numBoids % For each robot
                    if i == j
                        % Do Nothing
                    else
                        P{i,j} = zeros(3); % Correlated matrix
                    end
                end
            end
            % color particle initalization
            obj.color_particles = zeros(1,3);
            
            % beacon initialization
            obj.time_as_beacon = 0;
            obj.is_beacon = 0;
            
            
        end
        
        %% Boids functions-------------------------------------------------
        
        function obj = boids_update(obj,e_max,rho_max)
            
            % measure local density
            A = pi*obj.detection_range^2;
            rho = length(obj.neighbors) / A;
            covar = obj.covariance_e;
            
            mean_error = norm(obj.position_d(1:2) - obj.position_e(1:2));
            % update boids parameters
            obj.max_speed = (norm(covar)+2)/(norm(obj.position_e(1:2)-obj.position_e(1:2))+1)^2;
            if obj.max_speed > 5
                obj.max_speed = 5;
            end
            obj.Ka = rho/rho_max + mean_error/e_max;
            obj.Kc = (norm(covar) + mean_error^2)/A; %norm([norm(robot.covariance), mean_error^2]);
            obj.Ks = A/(norm(covar) + mean_error^2);
            obj.Kh = (mean_error^2 + norm(covar))/(norm(obj.home-obj.position_e(1:2))^2);
            obj.Kg = obj.detection_range/(norm(obj.goal-obj.position_e(1:2))*norm(covar));
            
        end
        
        
        function obj = apply_force(obj, sep_force, coh_force,  ali_force)
            home_force = obj.seek(obj.home);
            if obj.found_goal == 1
                obj.Kg = 0;
            end
            goal_force = obj.seek(obj.goal);
            obj.acceleration = sep_force+coh_force+ali_force+obj.Kh*home_force+ obj.Kg*goal_force;
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
        
        function [steer] = seek(obj, target)
            desired = target - obj.position_e(1:2);
            desired = desired/norm(desired);
            desired = desired*obj.max_speed;
            
            steer = desired-obj.velocity_e;
            steer = steer.*obj.max_force;
        end
        
        function [steer] = seperate(obj, boids)
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
        
        function steer = align(obj, boids)
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
        
        function steer = cohesion(obj, boids)
            neighbor_dist = 50;
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
        
        %% measurments-----------------------------------------------------
        
        function obj = lidar_measurement(obj,ROBOTS)
            
            dists = [];
            angles = [];
            numBots = length(ROBOTS);
            r = obj.ID;
            
            for L = 1:numBots % other robot
                
                %distance from r to L
                d = norm(ROBOTS(L).position_t(1:2)- ROBOTS(r).position_t(1:2)); %truth
                d = d + normrnd(0,ROBOTS(r).sigmaRange,1,1); %noise
                %angle from r to L
                phi = atan2(ROBOTS(L).position_t(2)- ROBOTS(r).position_t(2), ROBOTS(L).position_t(1)- ROBOTS(r).position_t(1)); % truth
                phi = phi + angdiff(ROBOTS(r).position_e(3), ROBOTS(r).position_t(3)); %bias
                phi = phi + normrnd(0,ROBOTS(r).sigmaHeading,1,1); %noise
                dists = [dists, d];
                angles = [angles, phi];
            end
            
            obj.laser = dists;
            obj.bearing = angles;
            
        end
        
        function obj = encoder_measurement(obj)
            obj.vel_m = norm(obj.velocity_t)+ normrnd(0,obj.sigmaVelocity,1,1) + obj.biasVelocity;
            obj.yaw_rate_m = obj.position_t(3)-obj.path_t(end-1,3) + normrnd(0,obj.sigmaYawRate,1,1) + obj.biasYawRate;
        end
        
        function obj = get_locations(obj, ROBOTS)
            
            neigh = [];
            numBots = length(ROBOTS);
            particles = obj.state_particles;
            for L = 1:numBots % other robot
                
                d = obj.laser(L);
                phi = obj.bearing(L)+pi;
                
                %if the other robot is in detection / communication range
                %give it our dead-reckoning prediction of where it is
                %update our particle if we have one there already
                if L == obj.ID
                    particles{1}(:,L) = [obj.position_e(1);obj.position_e(2);1];
                    particles{2}(:,:,L) = obj.covariance;
                elseif d < obj.detection_range
                    dx = d*cos(phi); %from r -> L
                    dy = d*sin(phi); %from r -> L
                    x0 = ROBOTS(L).position_e(1); %r pose
                    y0 = ROBOTS(L).position_e(2); %r pose
                    %give L, r's position of L, and mark that it has that
                    %particle
                    particles{1}(:,L) = [x0+dx;y0+dy;1];
                    particles{2}(:,:,L) = ROBOTS(L).covariance;
                    neigh = [neigh,ROBOTS(L)];
                else
                    particles{1}(:,L) = [0;0;0];
                    particles{2}(:,:,L) = [0,0;0,0];
                end
            end
            
            obj.state_particles = particles;
            obj.neighbors = neigh;
            
        end
        
        %% kinematic update------------------------------------------------
        
        function obj = update(obj)
            
            %determine if we are a beacon-------------------------------------------
            if obj.is_beacon == 1 && rand > obj.time_as_beacon/100 % remain a beacon
                obj.velocity_e = [0,0];
                obj.velocity_t = [0,0];
                obj.velocity_d = [0,0];
            elseif obj.is_beacon == 1 % stop becoming a beacon
                obj.is_beacon = 0;
            else                     % i am not a beacon
                
                % update truth velocity and position
                obj.velocity_t = obj.velocity_t + obj.acceleration;
                obj.velocity_t = obj.velocity_t./norm(obj.velocity_t).*obj.max_speed;
                ttheta = atan2(obj.velocity_t(2),obj.velocity_t(1));
                obj.position_t = [obj.position_t(1:2) + obj.velocity_t, ttheta];
                
                %perform dead_reckoning
                obj = obj.dead_reckoning();
                
                % update estimate of location
                obj = obj.estimate_location();
                
                % check if we can see home
                obj = home_update(obj.detection_range);
                
                %record paths
                obj.path_t = [obj.path_t; obj.position_t];
                obj.path_d = [obj.path_d; obj.position_d];
                obj.path_e = [obj.path_e; obj.position_e(1:2)];
                
                %set acceleration to zero
                obj.acceleration = [0 0];
                
                % check if we reached a goal or not
                if norm(obj.position_e(1:2) - obj.goal) < obj.detection_range
                    obj.found_goal = 1;
                end
                
            end
        end
        
        %% localization functions------------------------------------------
        
        function obj = estimate_location(obj)
            switch (obj.estimator)
                case 0 % just use dead_reckoning
                    obj.position_e = obj.position_d;
                    obj.velocity_e = obj.velocity_d;
                    obj.covariance_e = obj.covariance_d;
                case 1 % covariance intersection
                    obj = obj.covariance_intersection();
                case 2 % decentralized ekf
                    obj = obj.decentralized_ekf();
                case 3 % centralized ekf
            end
        end
        
        
        function obj = dead_reckoning(obj)
            % update dead reckoning
            new_theta = obj.position_d(3) + obj.yaw_rate_m;
            obj.velocity_d = obj.vel_m*[cos(new_theta), sin(new_theta)];
            obj.position_d = [obj.position_d(1:2) + obj.velocity_d, new_theta];
            
            F_d = [1,0,-obj.vel_m*sin(new_theta);  % X
                   0,1, obj.vel_m*cos(new_theta);  % Y
                   0,0,           1             ]; % Yaw
                 
            
            Q_d = diag(obj.sigmaVelocity, obj.sigmaVelocity, obj.sigmaYawRate/100);
            
            obj.covariance_d = F_d*obj.covariance_d*F_d' + Q_d;
        end
        
        function obj = covariance_intersection(obj)
            % measure error in variance and dead reckoning mean error
            if sum(obj.state_particles{1}(3,:)) > 1
                states = obj.state_particles{1}(1:2,obj.state_particles{1}(3,:) >.5);
                covars = obj.state_particles{2}(:,:,obj.state_particles{1}(3,:) > .5);
                
                [mean_pose,covar] = fusecovint(states,covars);
                obj.position_e = mean_pose';
                obj.covariance_e = covar;
                obj.velocity_e = obj.velocity_d;
            else
                obj.position_e = obj.position_d;
                obj.covariance_e = obj.covariance_d;
                obj.velocity_e = obj.velocity_d;
            end
        end
        
        function obj = decentralized_ekf(obj)
            
        end
        
        function obj = home_update(obj,home_range)
            
            home_dist = norm(obj.position_t(1:2) - obj.home);
            if home_dist < home_range
                % update location
                obj.covariance_d = [.01,.001;.001,.01];
                obj.covar_e = obj.covariance_d;
                obj.position_d = obj.t_position + normrnd(0,.001,1,3);
                obj.position_e = obj.position_d;
                
                obj.state_particles{1} = obj.state_particles{1}*0;
                obj.state_particles{2} = obj.state_particles{2}*0;
                
                
                %get color particles
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
        
        %% beacon functions------------------------------------------------
        
        % NEEED TO FIX!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        function ROBOTS = beacon_update(ROBOTS,ID, neighbors, cov_max)
            
            found_beacon = 0;
            beacon = [];
            % if norm(ROBOTS(ID).mean_covar) < cov_max
            %     ROBOTS(ID).is_beacon =0;
            % end
            
            if ROBOTS(ID).is_beacon == 1 && length(neighbors) <= 1
                ROBOTS(ID).time_as_beacon = ROBOTS(ID).time_as_beacon+1;
                found_beacon = 1;
                beacon = ID;
            end
            
            for r = neighbors
                if r.is_beacon == 1
                    found_beacon =1;
                    if length(neighbors) <= 1
                        ROBOTS(r.ID).time_as_beacon = ROBOTS(r.ID).time_as_beacon+1;
                    else
                        ROBOTS(r.ID).time_as_beacon = 0;
                    end
                    beacon = r.ID;
                    break;
                end
            end
            
            if found_beacon == 1
                for r = neighbors
                    if norm(r.mean_covar) > norm(ROBOTS(beacon).mean_covar) %may need to flip the sign
                        ROBOTS(beacon).is_beacon = 0;
                        ROBOTS(beacon).time_as_beacon = 0;
                        ROBOTS(r.ID).is_beacon = 1;
                        ROBOTS(r.ID).time_as_beacon = 0;
                        beacon = r.ID;
                    end
                end
            else
                if norm(ROBOTS(ID).mean_covar) > cov_max
                    ROBOTS(ID).is_beacon =1;
                    ROBOTS(ID).time_as_beacon = 0;
                end
                
            end
            
            
        end
        
        %% color particles functions---------------------------------------
        function [obj, other] = trade_color(obj, other)
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
        
        %% Display functions-----------------------------------------------
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
                quiver(ROBOTS(r).t_position(1),ROBOTS(r).t_position(2),ROBOTS(r).t_velocity(1), ROBOTS(r).t_velocity(2),'k');
                hold on;
                plot(ROBOTS(r).t_position(1), ROBOTS(r).t_position(2), 'ks');
                hold on;
                if show_detection_rng
                    viscircles([ROBOTS(r).position(1),ROBOTS(r).position(2)],range);
                    hold on;
                end
                %plot estimated position and color-------------------------------------------
                error_ellipse(ROBOTS(r).mean_covar, [ROBOTS(r).mean_position(1), ROBOTS(r).mean_position(2)])
                hold on;
                if sum(ROBOTS(r).color_particles) > 0
                    COLOR= ROBOTS(r).color_particles./sum(ROBOTS(r).color_particles);
                else
                    COLOR = [0,0,0];
                end
                
                if ROBOTS(r).is_beacon == 1
                    plot(ROBOTS(r).mean_position(1), ROBOTS(r).mean_position(2), '^', 'color', COLOR);
                    hold on;
                else
                    plot(ROBOTS(r).mean_position(1), ROBOTS(r).mean_position(2), '*', 'color', COLOR);
                    hold on;
                end
                % plot dead reckoning position---------------------------------------------------------
                plot(ROBOTS(r).position(1), ROBOTS(r).position(2), 'bo');
                hold on;
                quiver(ROBOTS(r).position(1), ROBOTS(r).position(2),ROBOTS(r).velocity(1), ROBOTS(r).velocity(2), 'b');
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
            axis([-50 50 -50 50])
            
            %plot gain distributions
            %     subplot(2,3,2)
            %     histogram(KA)
            %     title("alignment gain")
            %     subplot(2,3,3)
            %     histogram(KC)
            %     title("cohesion gain")
            %     subplot(2,3,4)
            %     histogram(KS)
            %     title("Seperation gain")
            %     subplot(2,3,5)
            %     histogram(KH)
            %     title("Home gain")
            %     subplot(2,3,6)
            %     histogram(KG)
            %     title("Goal gain")
            
            
            pause(.0001);
        end
        
        
    end
end
