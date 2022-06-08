classdef agent
    properties 
        ID
        home
        position
        velocity
        range
        show_range
        Ks
        Ka
        Kc
        path
        dt
        max_speed
        min_speed
        acceleration
        max_force
        neighbors_data
        neighbors
        

    end

    methods
        function obj=agent(x,y,Ks,Ka,Kc,ID)
            obj.ID=ID;
            %INITIALIZE
            angle = (2*pi).*rand;
            obj.velocity = [cos(angle), sin(angle)];
                     
            obj.position = [x, y];
                        
            obj.path = [x, y];
            
            %parameters
            obj.max_speed =0.5;
            obj.min_speed=0.1;
            obj.max_force=1.5;
            obj.acceleration=[0 0];
            

            obj.Ks = Ks;
            obj.Ka = Ka;
            obj.Kc = Kc;

        end 

        function obj=apply_force(obj,separation_force,cohesion_force,align_force)
            
            home_force=obj.seek(obj.home);
            
            force=separation_force+cohesion_force+align_force+20*home_force;
            obj.acceleration=obj.acceleration+force;
            
        end

        function obj = swarm(obj,boids)
            
            sep = obj.separate(boids);
            ali = obj.align(boids);
            coh = obj.cohesion(boids);
            
            sep = sep.*obj.Ks;
            ali = ali.*obj.Ka;
            coh = coh.*obj.Kc;
            
            obj=obj.apply_force(sep,coh,ali);
        end

        function obj=bounds(obj,map)
            xmin=map(1);
            xmax=map(2);
            ymin=map(3);
            ymax=map(4);

            if obj.position(1) < xmin || obj.position(2) < ymin
                obj.velocity= 200*(obj.seek(obj.home)); %replace obj velocity for steer and add to force application
            end
            if obj.position(1)>xmax || obj.position(2) > ymax
               obj.velocity= -200*(obj.seek(obj.home));
            end
            
        end

        function obj=update(obj)

%             new_theta = obj.position_d(3) + obj.yaw_rate_m*obj.dt;
%             obj.velocity_d = obj.vel_m*[cos(new_theta), sin(new_theta)];
%             obj.position_d = [obj.position_d(1:2) + obj.velocity_d*obj.dt, new_theta];

            obj.velocity=obj.velocity+obj.acceleration;
            obj.velocity = obj.velocity./norm(obj.velocity).*obj.max_speed;
            obj.position = obj.position + obj.velocity;
            obj.path=[obj.path;obj.position];
            obj.acceleration = [0 0];
        end
        
        function [steer] = seek(obj, target)
            desired = target - obj.position;
            desired = norm(desired);
            desired = desired*obj.max_speed;
            
            steer = desired-obj.velocity;
            steer = steer./norm(steer).*obj.max_force;
        end
        function [steer] = separate(obj, boids)
            desired_separation = obj.range;
            steer = [0,0];
            count = 0;
            agent_positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                agent_positions(:,i) = boids(i).position(1:2);
            end
            distance2agents = pdist([obj.position(1:2); agent_positions']); %compute distance to neighbourds
            distance2agents = distance2agents(1:length(boids));
%             obj.test=distance2agents;
            for i=1:1:length(boids)
                if distance2agents(i) > 0 && distance2agents(i) <  desired_separation
                    delta = obj.position(1:2) - boids(i).position(1:2);
                    delta = delta./norm(delta);
                    delta = delta./distance2agents(i);
                    steer = steer + delta;
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
            neighbor_dist = obj.range; %explore other constants
            sum = [0 0];
            count = 0;
            steer = [0 0];
            
            agent_positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                agent_positions(:,i) = boids(i).position(1:2);
            end
            distance2agents = pdist([obj.position(1:2); agent_positions']);
            distance2agents = distance2agents(1:length(boids));
            
            for i=1:1:length(boids)
                if distance2agents(i)>0 && distance2agents(i) < neighbor_dist
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
            neighbor_dist = obj.range;
            sum = [0 0];
            count = 0;
            steer = [0 0];
            
            agent_positions = zeros(2,length(boids));
            for i=1:1:length(boids)
                agent_positions(:,i) = boids(i).position(1:2);
            end
            distance2agents = pdist([obj.position(1:2); agent_positions']);
            distance2agents = distance2agents(1:length(boids));
            
            for i=1:1:length(boids)
                if distance2agents(i)>0 && distance2agents(i) < neighbor_dist
                    sum=sum+boids(i).position(1:2);
                    count=count+1;
                end
            end
            
            if count > 0
                sum=sum./count;
%                 sum=sum./norm(sum).*obj.max_speed;
%                 steer=sum-obj.velocity;
%                 steer=steer./norm(steer).*obj.max_force;
                steer = obj.seek(sum);
            end
        end
        function obj = get_neighbors(obj,AGENTS)
            
            distance2agents = []; % Distance to every agent
            angles = [];% Global angles to every agent
%             obj.Nagents = length(AGENTS);
            id = obj.ID;  % This agent
            neighborsID = [];  % other agents
            
            for N = 1:length(AGENTS) % other agents
                
                % Distance from this agent (id) to other agent (N)
                d = norm(AGENTS(N).position(1:2)- AGENTS(id).position(1:2)); 
                % Angle from id to N
                angle = atan2(AGENTS(N).position(2)- AGENTS(id).position(2), AGENTS(N).position(1)- AGENTS(id).position(1));
                
                distance2agents = [distance2agents, d];
                angles = [angles, angle];
                if d < 30
                    neighborsID = [neighborsID, AGENTS(N).ID];
                end
            end
            obj.neighbors_data=[distance2agents;angles];
            obj.neighbors = neighborsID;
            
        end
        function show_swarm(AGENTS,range,show_range,map)
            
            N_agents = length(AGENTS);
            clf();
            for r = 1:N_agents
           
                plot(AGENTS(r).path(end,1), AGENTS(r).path(end,2), 'bo');
                hold on;
                % show circles around agent to show range 
                if show_range
                    viscircles([AGENTS(r).path(end,1),AGENTS(r).path(end,2)],range);
                    hold on;
                end
                
            end
            % Map limits
            title("Swarm simulation");
            axis(map)
            pause(.0001);
            
        end


    end
                           
              

end




