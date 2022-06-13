classdef agent
    properties 
        ID
        home
        position
        velocity
        angle_rate
        max_delta_angle
        dt
        range
        show_range
        Ks
        Ka
        Kc
        path
        max_speed
        min_speed
        acceleration
        max_acceleration
        neighbors_data
        neighbors
        mapsize
        
        

    end

    methods
        function obj=agent(x,y,Ks,Ka,Kc,ID)
            obj.ID=ID;
            %INITIALIZE
            angle = (2*pi).*rand();

            obj.velocity = [cos(angle), sin(angle)];
                     
            obj.position = [x, y,angle];
                        
            obj.path = [x, y];

            obj.angle_rate=0;

            %parameters
            obj.max_speed =4;
            obj.min_speed=0.1;
            obj.max_acceleration=4*obj.dt;
            obj.max_delta_angle=1.5*pi()*obj.dt;          

            obj.Ks = Ks;
            obj.Ka = Ka;
            obj.Kc = Kc;

        end 

        function velocity_update=velocity_update(obj,boids)
            inertia=obj.move(boids);
            sep = obj.separate(boids);
            ali = obj.align(boids);
            coh = obj.cohesion(boids);
            
            sep = sep.*obj.Ks;
            ali = ali.*obj.Ka;
            coh = coh.*obj.Kc;
            
            velocity_update=obj.velocity+sep+coh+ali+2*inertia;
            
        end

        function obj=bounds(obj)
                    
            if obj.position(1) > obj.mapsize || obj.position(1) < -obj.mapsize
                if obj.velocity(1)< 0  && obj.position(1)< -obj.mapsize
                    obj.position(1)=obj.position(1) + 2*obj.mapsize;
                elseif obj.velocity(1)> 0  && obj.position(1)> obj.mapsize
                    obj.position(1)=obj.position(1) - 2*obj.mapsize;
                end
            
            end
            if obj.position(2) > obj.mapsize || obj.position(2) < -obj.mapsize
                if obj.velocity(2)< 0  && obj.position(2)< -obj.mapsize
                    obj.position(2)=obj.position(2) + 2*obj.mapsize;
                elseif obj.velocity(2)> 0  && obj.position(2)> obj.mapsize
                    obj.position(2)=obj.position(2) - 2*obj.mapsize;
                end
            
            end                    
        end

        function obj=update(obj,boids)
            obj=obj.bounds();
            velocity_update=obj.velocity_update(boids);

            if norm(velocity_update)>0 && norm(obj.velocity)>0
                theta=atan2(obj.velocity(2),obj.velocity(1));
                new_theta=atan2(velocity_update(2),velocity_update(1));
                theta_error=angdiff(new_theta,theta);
%                 theta_error=new_theta-theta;
                if abs(theta_error) > obj.max_delta_angle
                    v=obj.velocity/norm(obj.velocity);
                    v=v*norm(velocity_update);
                    if theta_error > 0 % for 0 to pi (positive angle)
                        v=[v*cos(obj.max_delta_angle) -v*sin(obj.max_delta_angle)]; %[dvx -dvy]
                    elseif theta_error < 0 % for 0 to -pi (negative angle)
                        v=[v*cos(obj.max_delta_angle) v*sin(obj.max_delta_angle)]; %[dvx dvy]
                    end 
                    
                    velocity_update=v;
                end
            end

            if norm(velocity_update)-norm(obj.velocity) > obj.max_acceleration
                if norm(velocity_update)==0
                    velocity_update=obj.velocity+(obj.max_acceleration*obj.velocity/norm(obj.velocity));
                else
                    velocity_update=velocity_update*((obj.max_acceleration+obj.velocity)/norm(velocity_update));
                end
            elseif norm(velocity_update)-norm(obj.velocity) > -obj.max_acceleration
                if norm(velocity_update)==0
                    velocity_update=obj.velocity-(obj.max_acceleration*obj.velocity/norm(obj.velocity));
                else
                    velocity_update=velocity_update*((-obj.max_acceleration+obj.velocity)/norm(velocity_update));
                end
            end

            if norm(velocity_update)>obj.max_speed
                velocity_update=velocity_update*(obj.max_speed/norm(velocity_update));
            end

            obj.velocity=velocity_update;
            obj.position(1:2)=obj.position(1:2)+obj.velocity*obj.dt;
            obj.path=[obj.path;obj.position(1:2)];           
                        
        end
        function [agent_states]=get_agents_states(obj,boids)
%follows the format:   agent_states=[dist2agent x y vx vy] for each agent within r
            neighbor_range = obj.range;
            agent_states=[];
            agent_positions = zeros(length(boids),2);
            agent_velocities = zeros(length(boids),2);
            for i=1:1:length(boids)
                agent_positions(i,:) = boids(i).position(1:2);
                agent_velocities(i,:) = boids(i).velocity;
            end
            distance2agents = pdist([obj.position(1:2); agent_positions]);
            distance2agents = distance2agents(1:length(boids));
            for i=1:1:length(boids)
                if distance2agents(i)>0 && distance2agents(i) < neighbor_range
                    agent_states=[distance2agents(i),agent_positions(i,:),agent_velocities(i,:)];
                end
            end

        end
        function [steer] = seek(obj, target)
            desired = target - obj.position(1:2);
            desired = norm(desired);
            desired = desired*obj.max_speed;
            
            steer = desired-obj.velocity;
            steer = steer./norm(steer).*obj.max_force;
        end
        function [steer] = separate(obj, boids)
            steer = [0 0];
            agents_states=obj.get_agents_states(boids);
            if ~isempty(agents_states)
                position_diff = obj.position(1:2) - agents_states(:,2:3);
                norm_diff=position_diff./agents_states(:,1);
                steer = sum(-norm_diff.*(1/agents_states(:,1).^2));
            end
        end
        function steer = align(obj, boids)
%             neighbor_range = obj.range; %explore other constant
            steer = [0 0];
            agents_states=obj.get_agents_states(boids);
            if ~isempty(agents_states)
                velocity_sum=sum(agents_states(:,4:5))./length(agents_states(:,1));
                steer=velocity_sum;
            end
            
        end
        function steer = cohesion(obj, boids)
%             neighbor_range = obj.range;
            steer = [0 0];
            agents_states=obj.get_agents_states(boids);
            if ~isempty(agents_states)
                position_sum=sum(agents_states(2:3))./length(agents_states(:,1));
                steer=position_sum;
            end
        end
        function steer = move(obj,boids)
            %keep moving in the direction if there are no agents close
            steer = [0 0];
            agents_states=obj.get_agents_states(boids);
            if isempty(agents_states)
                steer=obj.velocity;
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
        function show_swarm(AGENTS,range,show_range)
            x=AGENTS.mapsize;
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
            axis([-x x -x x])
            pause(.001);
            
        end


    end
                           
              

end



