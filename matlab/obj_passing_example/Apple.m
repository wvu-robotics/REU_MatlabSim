classdef Apple < handle
    %APPLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = Apple()
            %APPLE Construct an instance of this class
            %   Detailed explanation goes here
        end
      
        function change_passed_obj(obj,obj_in)
            obj_in.x = 5;
            
        end
    end
end

