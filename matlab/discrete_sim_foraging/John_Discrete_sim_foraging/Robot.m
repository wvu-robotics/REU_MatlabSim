classdef Robot < handle
    %ROBOT Summary of this class goes here
    %One robot object for each simulated robot. The robot decides how it
    %will move and returns it's actions to the owning class.
    properties
        id = 0;
        x = 1;
        y = 1;
        wallet = 2;
        battery =3;
        last_heading;
        local_map;
        local_scan;
        scan_range = 5;
        fog_of_war;
        move_state; %use this to store if we should be doing a levy jump etc
        obj_in = World.empty;
    end
    
    methods
        function obj = Robot(id,x,y,wallet,battery,heading,map_in)
            %ROBOT Construct an instance of this class
            obj.x = x; %init x coord
            obj.y = y;
            obj.id = id; %A unique ID for the robot
            obj.wallet = wallet; %init amount of money
            obj.battery = battery;
            
            obj.local_map = map_in;
           %obj.figure1 = figure;
            obj.fog_of_war = zeros(110,110,4); 



            
            if (heading == 0) %If we start with no specified init direction pick a random one
                obj.last_heading = randi([1,8]);
            else
                obj.last_heading = heading;
            end
            
        end
        
        %This will call other functions depending on if the robot should be
        %foraging, moving home, moving to trade, etc.
        function move_next()
            %Are we searching?
            %move_search();
            
            %Are we going home?
            %move_home();
        end
        
        function xy = get_xy(obj)
            %Returns a tuple of the robots present X and Y coords
            xy = [obj.x,obj.y];
        end

        
        function xy = move(obj)
            %TODO split this into multiple funcs for different behaviours!
            %use persistence
            %check for boundaries
            %move the robot in a random direction
            disp("moving robot");
            temp_x = obj.x;
            temp_y = obj.y;
            
            step_size = 1;
            
            dir = obj.last_heading + randi([-3,3]);
            dir = mod(dir,8);%wrap around heading value
            if(dir == 0)
                dir = 1;
            end
            obj.last_heading = dir;
            disp(dir);
            
            %hacky levy walk implementation, spin out to new function
            chance = rand;
            if( chance < 0.5)
                step_size = 5;
            end
            
            dir = randi([1,8]);
            obj.last_heading = dir;
            
            
            
            if dir == 1
                obj.x = obj.x;
                obj.y = obj.y+step_size;
            elseif (dir == 2)
                obj.x = obj.x-step_size;
                obj.y = obj.y+step_size;
            elseif (dir == 3)
                obj.x = obj.x-step_size;
                obj.y = obj.y;
            elseif (dir == 4)
                obj.x = obj.x-step_size;
                obj.y = obj.y-step_size;
            elseif (dir == 5)
                obj.x = obj.x;
                obj.y = obj.y-step_size;
            elseif (dir == 6)
                obj.x = obj.x+step_size;
                obj.y = obj.y-step_size;
            elseif (dir == 7)
                obj.x = obj.x+step_size;
                obj.y = obj.y;
            elseif (dir == 8)
                obj.x = obj.x+step_size;
                obj.y = obj.y+step_size;
            end           
            
            %pick up food
%             if(obj.map(obj.robot_x, obj.robot_y, 2) >0)
%                 obj.map(obj.robot_x, obj.robot_y, 2) = 0;
%                 disp("Picked up food!");
%             end
            
            %Guard leaving map
            if(obj.x < 1)
                obj.x = 1;
            end
            if (obj.x > size(obj.local_map.map,1))
                obj.x = size(obj.local_map.map,1);
            end
            if(obj.y < 1)
                obj.y = 1;
            end
            if(obj.y > size (obj.local_map.map,1))
                obj.y = size(obj.local_map.map,1);
            end

            if(obj.local_map.map(obj.x, obj.y, 2) >0)
                obj.x = temp_x;
                obj.y = temp_y;
                %disp("oopsy woopsy, I crashed!");
            end
           xy = [obj.x, obj.y];

        end
        
        function lscan(obj,obj_in)
            obj.local_scan = obj_in.scan(obj.x,obj.y,obj.scan_range);
%             pcolor(obj.local_scan(:,:,4));
%             imagesc(obj.local_scan(:,:,4));
%             text(obj.scan_range + 1,obj.scan_range + 1,string(obj.id))
            %axis off;
            
        end
        
        function fscan(obj,obj_in)
            holder = obj.local_scan(:,:,2)*1 + obj.local_scan(:,:,3)*2 + (obj.local_scan(:,:,4) & ones(size(obj.local_scan,1))) *7;
            obj.fog_of_war(obj.x - obj.scan_range: obj.x + obj.scan_range, obj.y - obj.scan_range: obj.y + obj.scan_range,4) = holder;
            pcolor(obj.fog_of_war(:,:,4));
            axis image;
            
        end
        
        
        
     end
end

