classdef World < handle & matlab.mixin.Copyable % handle to make objects callable by reference?
    
    %properties to hold handles need to be initiated in the way below
    properties (Access = public)
        max_x;
        max_y;
        draw_map %The map which gets drawn
        robot_list = Robot(0,1,1,0,0,0, World_Map("world_image_1.png")); %Holds all our robot objects
        num_robots =10;
        worldMap;% = World_Map("world_image_1.png"); %data structure to hold the world (3d matrix)
    end
    
    methods (Access = public)
        function obj = World(file_name)
           %Constructor, make a new world and set up the object's variables
           %file_name is the name of the file holding the bitmap of the obstacle layer of the world
           disp("Building World!");
           
           obj.worldMap = World_Map(file_name);
           obj.worldMap.gen_map(file_name);
           obj.worldMap.max_x = size(obj.worldMap.map,1);
           obj.worldMap.max_y = size(obj.worldMap.map,2);
           
           obj.max_x = size(obj.worldMap.map,1);
           obj.max_y = size(obj.worldMap.map,2);
           %obj.robot_list(1:10,1) = Robot(0,1,1,1,0,0, World_Map("world_image_1.png"));
           
         end
        
%         function draw_world(obj)
%             %Draw out the world to our figure
%             %Loop through our world and plot each layer with appropriate
%             %symbols   
%             drawing = obj.map(:,:,1) + obj.map(:,:,2) + obj.map(:,:,3) + obj.map(:,:,4);
%             pcolor(drawing);
%         end
        
        function gen_food(obj,num_locs, density)
            %Assume layer 2 of world is for food
            %Generate a matrix of food locations using 
            %TODO make num_locs and density do something
            disp("Placing food!");
            food_map = randsrc(obj.max_x, obj.max_y,[1,0;0.01,0.99]);
            %obj.map(:,:,2) = food_map & (xor(food_map, obj.map(:,:,1)));
            food_map = food_map & (xor(food_map,obj.worldMap.map(:,:,1)));
            %food_map = food_map & (xor(food_map,obj.worldMap.map(:,:,3)));
            obj.worldMap.map(:,:,2) = food_map *2;
            %obj.worldMap.map
        end
        
        function gen_bots(obj, num)
           %Create a list of our robots with default values of zero
           %each robot will get a copy of the world map
           disp("Generating bots!");
           obj.num_robots = num;
           for i=1:obj.num_robots %set up each robot object with a ID and a map object
           obj.robot_list(i,1) = Robot(i,40,40,1,0,0, World_Map("world_image_1.png"));
           obj.robot_list(i,1).local_map.map(:,:,1) = obj.worldMap.map(:,:,1);
           end
           %obj.robot_list(1).local_map.map(1,1,1) = 1; %change (1,1) of first robot)
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
                %get the present XY of the robot
                curr_xy = obj.robot_list(ii).get_xy();
                %disp("robot xy" + curr_xy);
                %set the robots current position to zero
                obj.worldMap.map(curr_xy(1),curr_xy(2),3) = 0;
                %Get the new XY where the robot moved to
                new_xy = obj.robot_list(ii).move();
                if (obj.worldMap.map(new_xy(1),new_xy(2),2) > 0)
                    obj.worldMap.map(new_xy(1),new_xy(2),2) = 0;
%                     [j, Fs] = audioread('/home/z/doh.mp3');
%                     sound(j, Fs, 16);
%                     clear sound;
                end
                %Set the new XY to the ID number of the robot
                obj.worldMap.map(new_xy(1), new_xy(2),3) = obj.robot_list(ii).id;
            end
            obj.worldMap.draw_world();
 
        end
            
            
        
    end
end

