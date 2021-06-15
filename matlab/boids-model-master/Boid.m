classdef Boid
    
    properties
        ID
        position
        velocity
        acceleration
        r
        max_force
        max_speed
        Ks
        Ka
        Kc
        path
        laser
        bearing
        particles
        neighbors
        detection_range
    end
    
    methods
        function obj = Boid(position_x,  position_y, Ks, Ka,Kc, numBoids, ID)
            obj.ID = ID;
            obj.acceleration = [0 0];
            
            angle = (2*pi).*rand;
            obj.velocity = [cos(angle), sin(angle)];
            
            obj.position = [position_x, position_y];
            obj.r = 0;
            obj.max_speed = 2;
            obj.max_force = 0.1;
            
            obj.Ks = Ks;
            obj.Ka = Ka;
            obj.Kc = Kc;
            
            %obj.path = zeros(time,4);
            obj.path = [position_x, position_y, obj.velocity(1), obj.velocity(2),0];
            obj.laser = zeros(1,numBoids);
            obj.particles = zeros(3,numBoids);
        end
        
        
        function obj = apply_force(obj, sep_force, coh_force,  ali_force)
            obj.acceleration = obj.acceleration+sep_force+coh_force+ali_force;
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
            vel_old = obj.velocity;
            obj.velocity = obj.velocity + obj.acceleration;
            obj.velocity = obj.velocity./norm(obj.velocity).*obj.max_speed;
            obj.position = obj.position + obj.velocity;
            obj.acceleration = [0 0];
            
            omega = atan2(obj.velocity(2)-vel_old(2),obj.velocity(1)-vel_old(1));
            obj.path = [obj.path;[obj.position + obj.velocity, obj.velocity./norm(obj.velocity).*obj.max_speed, omega]];
            obj.particles(1,:) = obj.particles(1,:) + obj.velocity(1);
            obj.particles(2,:) = obj.particles(2,:) + obj.velocity(2);
        end
        
        function [steer] = seek(obj, target)
            desired = target - obj.position;
            desired = norm(desired);
            desired = desired*obj.max_speed;
            
            steer = desired-obj.velocity;
            steer = steer./norm(steer).*obj.max_force;
        end
        
        function [steer] = seperate(obj, boids)
            desired_separation = 100.0; %%%%%%%%%% communication range
            steer = [0,0];
            count = 0;
            positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                positions(:,i) = boids(i).position;
            end
            d = pdist([obj.position; positions']);
            d = d(1:length(boids));
            
            for i=1:1:length(boids)
                if d(i) > 0 && d(i) <  desired_separation
                    difference = obj.position - boids(i).position;
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
            neighbor_dist = 50;
            sum = [0 0];
            count = 0;
            steer = [0 0];
            
            positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                positions(:,i) = boids(i).position;
            end
            d = pdist([obj.position; positions']);
            d = d(1:length(boids));
            
            for i=1:1:length(boids)
                if d(i)>0 && d(i) < neighbor_dist
                    sum=sum+boids(i).position;
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
                positions(:,i) = boids(i).position;
            end
            d = pdist([obj.position; positions']);
            d = d(1:length(boids));
            
            for i=1:1:length(boids)
                if d(i)>0 && d(i) < neighbor_dist
                    sum=sum+boids(i).position;
                    count=count+1;
                end
            end
            
            if count > 0
                sum=sum./count;
                steer = obj.seek(sum);
            end
        end
    end
end
