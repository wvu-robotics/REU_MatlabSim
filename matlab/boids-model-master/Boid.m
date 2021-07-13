classdef Boid
    
    properties
        ID
        
        position  %my dead reckoning
        velocity
        acceleration
        covariance
        
        r
        max_force
        max_speed
        Ks
        Ka
        Kc
        Kh
        Kg
        path
        laser
        bearing
        particles
        color_particles
        neighbors
        detection_range
        
        mean_position  %estimated from covariance intersection
        mean_covar
        mean_path 
        
        
        home
        goal
        found_goal
        is_beacon
        time_as_beacon
        invaders
        Isinvader
        
        sigmaVelocity
        sigmaYawRate
        biasVelocity
        biasYawRate
        sigmaRange
        sigmaHeading
        
        t_position  %truth 
        t_velocity
        t_path        
        
    end
    
    methods
        function obj = Boid(position_x,  position_y, Ks, Ka,Kc, numBoids, ID)
            obj.ID = ID;
            obj.acceleration = [0 0];
            
            angle = (2*pi).*rand;
            obj.velocity = [cos(angle), sin(angle)];
            obj.t_velocity = obj.velocity;
            
            obj.position = [position_x, position_y, angle];
            obj.t_position = obj.position;
            obj.r = 0;
            obj.max_speed = 5;
            obj.max_force = 0.1;
            
            obj.Ks = Ks;
            obj.Ka = Ka;
            obj.Kc = Kc;
            obj.Kh = 1;
            obj.Kg = 0;
            obj.found_goal = 1;
            
            
            obj.path = [position_x, position_y, angle];
            obj.t_path =  obj.path;
            obj.mean_path = obj.path(1:2);
            obj.laser = zeros(1,numBoids);
            obj.particles = cell(2,1);
            obj.particles{1} = zeros(3,numBoids);
            obj.particles{2} = zeros(2,2,numBoids);
            
            obj.color_particles = zeros(1,3);
            obj.time_as_beacon = 0;
            obj.is_beacon = 0;
            
            obj.invaders = [1;3;2;4];

            obj.Isinvader = 0;
            
        end
        
        
        function obj = apply_force(obj, sep_force, coh_force,  ali_force)
            home_force = obj.seek(obj.home);
            if obj.found_goal == 1
                obj.Kg = 0;
            end
            goal_force = obj.seek(obj.goal);
            %obj.velocity
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
        
        function obj = borders(obj, lattice_size)
            if obj.position(1) < -obj.r
                obj.position(1)=lattice_size(1)+obj.r;
            end
            
            if obj.position(2) < -obj.r
                obj.position(2)=lattice_size(2)+obj.r;
            end
            
            if obj.position(1) > lattice_size(1) + obj.r
                obj.position(1)=-obj.r;
            end
            
            if obj.position(2) > lattice_size(2) + obj.r
                obj.position(2)=-obj.r;
            end
        end
        
        function obj = update(obj)

            %determine if we are a beacon-------------------------------------------
            if obj.is_beacon == 1 && rand > obj.time_as_beacon/100 % remain a beacon
                obj.velocity = [0,0];
                obj.t_velocity = [0,0];
            elseif obj.is_beacon == 1 % stop becoming a beacon
                obj.is_beacon = 0;
            else                     % i am not a beacon
                old_pose = obj.t_position;
                
                % update truth velocity and position
                obj.t_velocity = obj.t_velocity + obj.acceleration;
                obj.t_velocity = obj.t_velocity./norm(obj.t_velocity).*obj.max_speed;
                ttheta = atan2(obj.t_velocity(2),obj.t_velocity(1));
                obj.t_position = [obj.t_position(1:2) + obj.t_velocity, ttheta];
                
                % update dead reckoning
                new_vel = norm(obj.t_velocity)+ normrnd(0,obj.sigmaVelocity,1,1) + obj.biasVelocity;
                omega = obj.t_position(3)-old_pose(3) + normrnd(0,obj.sigmaYawRate,1,1) + obj.biasYawRate;
                new_theta = omega + obj.position(3);
                obj.velocity = new_vel*[cos(new_theta), sin(new_theta)];
                obj.position = [obj.position(1:2) + obj.velocity, new_theta];
                
                noise1 = abs((obj.sigmaVelocity + obj.biasVelocity));%*max(cos(new_theta+obj.sigmaYawRate + obj.biasYawRate),cos(new_theta-obj.sigmaYawRate + obj.biasYawRate)));
                noise2 = abs((obj.sigmaVelocity + obj.biasVelocity));%*max(sin(new_theta+obj.sigmaYawRate + obj.biasYawRate),sin(new_theta-obj.sigmaYawRate + obj.biasYawRate)));
                obj.covariance = obj.covariance + [noise1,.01;.01,noise2];
                
                % update estimate of location
                % measure error in variance and dead reckoning mean error
                if sum(obj.particles{1}(3,:)) > 1
                    states = obj.particles{1}(1:2,obj.particles{1}(3,:) >.5);
                    covars = obj.particles{2}(:,:,obj.particles{1}(3,:) > .5);
                    
                    [mean_pose,covar] = fusecovint(states,covars);
                    obj.mean_position = mean_pose';
                    obj.mean_covar = covar;
                else
                    obj.mean_position = obj.position;
                    obj.mean_covar = obj.covariance;
                end
            end
            
            %record paths
            obj.t_path = [obj.t_path; obj.t_position];
            obj.path = [obj.path; obj.position];
            obj.mean_path = [obj.mean_path; obj.mean_position(1:2)];
            
            %set acceleration to zero
            obj.acceleration = [0 0];
            
            % check if we reached a goal or not
            if norm(obj.mean_position(1:2) - obj.goal) < obj.detection_range
                obj.found_goal = 1;
            end
            
        end
        
        function [steer] = seek(obj, target)
            desired = target - obj.position(1:2);
            %desired = desired/norm(desired);
            desired = desired*obj.max_speed;
            
            steer = desired-obj.velocity;
            steer = steer.*obj.max_force;
        end
        
        function [steer] = seperate(obj, boids)
            desired_separation = obj.detection_range; %%%%%%%%%% communication range
            steer = [0,0];
            count = 0;
            positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                positions(:,i) = boids(i).position(1:2);
            end
            d = pdist([obj.position(1:2); positions']);
            d = d(1:length(boids));
            
            for i=1:1:length(boids)
                if d(i) > 0 && d(i) <  desired_separation
                    difference = obj.position(1:2) - boids(i).position(1:2);
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
                    steer = steer - obj.velocity;
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
                positions(:,i) = boids(i).position(1:2);
            end
            d = pdist([obj.position(1:2); positions']);
            d = d(1:length(boids));
            
            for i=1:1:length(boids)
                if d(i)>0 && d(i) < neighbor_dist
                    sum=sum+boids(i).position(1:2);
                    count=count+1;
                end
            end
            
            if count > 0
                sum=sum./count;
                sum=sum./norm(sum).*obj.max_speed;
                steer=sum-obj.velocity;
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
                positions(:,i) = boids(i).position(1:2);
            end
            d = pdist([obj.position(1:2); positions']);
            d = d(1:length(boids));
            
            for i=1:1:length(boids)
                if d(i)>0 && d(i) < neighbor_dist
                    sum=sum+boids(i).position(1:2);
                    count=count+1;
                end
            end
            
            if count > 0
                sum=sum./count;
                steer = obj.seek(sum);
            end
        end
        
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
        
    end
end
