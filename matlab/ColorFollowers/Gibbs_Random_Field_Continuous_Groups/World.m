classdef World < handle % handle to make objects callable by reference
    
    %properties to hold handles need to be initiated in the way below
    properties (Access = public)
        max_x;
        max_y;
        draw_map %2D matrix for drawing
        num_robots;
        robot_list = Robot.empty;%Holds all our robot objects
        timeStep;
        
        
        draw_layer = 1;
        obs_layer = 2;
        food_layer = 3;
        robot_layer = 4;
        
    end
    
    methods (Access = public)
        function obj = World(timeStep, worldSize)
           %Constructor, make a new world and set up the object's variables
           %file_name is the name of the file holding the bitmap of the obstacle layer of the world
           disp("Building World!");
           
           obj.timeStep = timeStep;
           obj.max_x = worldSize;
           obj.max_y = worldSize;           
        end

        
        function gen_bots(obj, num, numColors)
           %Create a list of our robots with default values of zero
           %each robot will get a copy of the world map.
           disp("Generating bots!");
           obj.num_robots = num;
           for i=1:obj.num_robots %set up each robot object with a ID and a map object
           obj.robot_list(i,1) = Robot(i,randi(obj.max_x-1),randi(obj.max_y-1),obj.timeStep,randi(numColors));
           obj.robot_list(i,1).randomGoal(obj);
           end
        end
        
        
        function drawWorld(obj)
            figure(1)
            clf
            hold on
            for i = 1:obj.num_robots
                xlim([0,101]);
                ylim([0,101]);
                RGB = obj.robot_list(i).RGB;
                %RGB = RGB/256;
                plot(obj.robot_list(i).truex, obj.robot_list(i).truey, '.', 'MarkerEdge', RGB, 'MarkerSize', 25);
                set(gca,'Color','k');
            end
            hold off
        end
        
        function tick(obj)
            disp("num_robots " + obj.num_robots);
            for ii=1:obj.num_robots %make each robot bust a move
                obj.robot_list(ii).move(obj);
                obj.robot_list(ii).FindNewColor;
                if obj.robot_list(ii).CheckGoalAllowance()
                    obj.robot_list(ii).randomGoal(obj);
                end
            end
            obj.drawWorld();
        end 
    end
end

