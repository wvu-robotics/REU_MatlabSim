classdef Pear < handle
    %PEAR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x;
    end
    
    methods
        function obj = Pear()
            %Set our field to 10
            obj.x = 10;
            disp("x at start up: " + obj.x);
            %Make a new object
            child_obj = Apple();
            
            child_obj.change_passed_obj(obj)
            disp("x after passing ourselves: " + obj.x);
        end
       
    end
end

