classdef TempClass
    properties
        data
    end
    
    methods
        function obj = TempClass(test1,test2,test3)
            obj.data = [test1,test2,test3];
            fprintf("TempClass Constructed.\n");
        end
        
        function [test1, test2, test3] = unpackData(obj)
            objData = obj.data;
            test1 = objData(1);
            test2 = objData(2);
            test3 = objData(3);
        end
        
        function value = get.data(obj)
            value = obj.data;
            fprintf("Accessed: data = [%g,%g,%g]\n",obj.data(1),obj.data(2),obj.data(3));
        end
    end
end