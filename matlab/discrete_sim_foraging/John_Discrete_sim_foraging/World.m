classdef World < handle & matlab.mixin.Copyable % handle to make objects callable by reference
    
    %properties to hold handles need to be initiated in the way below
    properties (Access = public)
        max_x;
        max_y;
        draw_map %2D matrix for drawing
        num_robots;
        robot_list = Robot.empty;%Holds all our robot objects
        worldMap = World_Map.empty;%Holds the true world map
        bitmap = "What.png";
      
        
        draw_layer = 1;
        obs_layer = 2;
        food_layer = 3;
        robot_layer = 4;
        
    end
    
    methods (Access = public)
        function obj = World(file_name)
           %Constructor, make a new world and set up the object's variables
           %file_name is the name of the file holding the bitmap of the obstacle layer of the world
           disp("Building World!");
           
           obj.worldMap = World_Map(file_name);
           obj.worldMap.gen_map();
           
           obj.max_x = size(obj.worldMap.map,1);
           obj.max_y = size(obj.worldMap.map,2);           
        end
        
        function gen_food(obj, density)
            %Generate a matrix of food locations randomly. A lower density
            %means less food.
            disp("Placing food!");
            food_map = randsrc(obj.max_x, obj.max_y,[1,0;density,1-density]);
            food_map = food_map & (xor(food_map,obj.worldMap.map(:,:,obj.obs_layer)));
            obj.worldMap.map(:,:,obj.food_layer) = food_map *2;
        end
        
        function gen_bots(obj, num)
           %Create a list of our robots with default values of zero
           %each robot will get a copy of the world map.
           disp("Generating bots!");
           obj.num_robots = num;
           for i=1:obj.num_robots %set up each robot object with a ID and a map object
           obj.robot_list(i,1) = Robot(i,40,40,1,0,1, World_Map(obj.bitmap));
           obj.robot_list(i,1).local_map.map(:,:,obj.obs_layer) = obj.worldMap.map(:,:,obj.obs_layer);
           end
        end
        
        function place_bots(obj,x,y)
            %Place bots around an X-Y coord
            disp("Placing bots!");
            xy = [randperm(10,obj.num_robots)'+x, randperm(10,obj.num_robots)'] + [x,y];
            for i=1:obj.num_robots
                obj.robot_list(i).x = xy(i,1); %set the X coord of robot i
                obj.robot_list(i).y = xy(i,2);
            end
        end
            
        function tick(obj)
            disp("num_robots " + obj.num_robots);
            for ii=1:obj.num_robots %make each robot bust a move
                %get the present XY of the 
      
                curr_xy = obj.robot_list(ii).get_xy();
                %set the robots current position to zero
                obj.worldMap.map(curr_xy(1),curr_xy(2),obj.robot_layer) = 0;
                %Get the new XY where the robot moved to
                new_xy = obj.robot_list(ii).move();
                if (obj.worldMap.map(new_xy(1),new_xy(2),obj.food_layer) > 0)
                    obj.worldMap.map(new_xy(1),new_xy(2),obj.food_layer) = 0;
                end
                %Set the new XY to the ID number of the robot
                obj.worldMap.map(new_xy(1), new_xy(2),obj.robot_layer) = obj.robot_list(ii).id;
            end
            subplot(1,2,1); 
            obj.worldMap.draw_world();
            for jj=1:1 %obj.num_robots
               subplot(1,2,2)
               obj.robot_list(jj).lscan(obj);
               obj.robot_list(jj).fscan(obj);
                
             end
        end 
        
        function return_scan = scan(obj,rx,ry,scan_range)
             return_scan = obj.worldMap.map(rx-scan_range:rx+scan_range,ry-scan_range:ry+scan_range,:);
                disp(return_scan)
        end
        
        
    end
end

