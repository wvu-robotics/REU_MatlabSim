classdef World_Map < handle & matlab.mixin.Copyable
    %Holds the data structure representing the world and provides get/set
    %methods to manipulate said data structure.
    %The data structure can be imagined as a large 3D cube with the X-Y
    %axes mapping to the X-Y world coordinates and the Z axis mapping to
    %the type of object being stored.
    
    %For example a '1' value at (4,5,2) would mean there is a piece of food
    % at coordinates (4,5)
    
    properties
        map;
        max_x;
        max_y;
        layers = 4;
        ones_map;
        
        file_name;
        
        draw_layer = 1;
        obs_layer = 2;
        food_layer = 3;
        robot_layer = 4;
        
    end
    
    methods
        function obj = World_Map(file_name)
            %3D matrix to hold the world 
            obj.file_name = file_name;
            obj.map = zeros(size(imread(file_name),1),size(imread(file_name),2),obj.layers); 
            %Make a map of ones the same size as our world (useful for
            %masking later)
            obj.ones_map = ones(size(obj.map,1),size(obj.map,2));
            obj.max_x = size(obj.map,1);
            obj.max_y = size(obj.map,2);
        end
        
        function draw_world(obj)
            %Sum all the layers together and draw them 
            drawing = obj.map(:,:,obj.obs_layer)*1 + obj.map(:,:,obj.food_layer)*2 + (obj.map(:,:,obj.robot_layer) & obj.ones_map) *7;
            figure(1);
            imagesc(drawing);
            colormap(colorcube);
            pcolor(drawing);
            axis image;

        end
        
        function gen_map(obj)
            %Generate the obstacle layer. Returns to the obj.map to set it
            disp("MAP: Building obstacles!");
            obj.map(:,:,obj.obs_layer) = imread(obj.file_name); %add in the obstacles
        end
        
        function spawn_food_map(obj)
            %Spawn food in using a bitmap
            
        end
        
        function spawn_food_rand(obj,density)
            %Spawn food in randomly
        
        end


        
    end
end

