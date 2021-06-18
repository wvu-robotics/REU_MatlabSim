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
    end
    
    methods
        function obj = World_Map(file_name)
            %3D matrix to hold the world 
            obj.map = zeros(100,100,4); %TODO MAKE THESE NOT HARDCODED!!!!!!
            %gen_map(file_name);
        end
        
        function draw_world(obj)
            %sum all the layers together TODO(remove weights?)
            drawing = obj.map(:,:,1)*1 + obj.map(:,:,2)*2 + obj.map(:,:,3) *3;
            %ax.YDir = 'normal'; %I have no idea what this does
            pcolor(drawing);            
        end
        
        function gen_map(obj,file_name)
            %Generate the obstacle slayer. Returns to the obj.map to set it
            disp("MAP: Building obstacles!");
            obj.map(:,:,1) = imread(file_name);
            obj.max_x = size(obj.map,1);
            obj.max_y = size(obj.map,2);
             disp("max x " + obj.max_x);

        end
        
        function spawn_food
            
        end
        


        
    end
end

