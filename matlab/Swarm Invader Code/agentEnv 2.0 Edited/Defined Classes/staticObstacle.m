classdef staticObstacle < handle    
    properties (Access = public)
       pose = 0;
       heading; 
       broadCollisionSpace;

    end
    properties (Access = private)
       shape;
       shapeID;
       id;
    end
    
    methods
        function obj = staticObstacle(shape, pose, heading, id)
            obj.shape = shape;
            obj.pose = pose;
            obj.heading = heading; 
            X = [min(shape(:,1)) max(shape(:,1))];
            Y = [min(shape(:,2)) max(shape(:,2))];
            obj.broadCollisionSpace = [X(1),Y(1);
                                       X(1),Y(2);
                                       X(2),Y(2);
                                       X(2),Y(1)];
            obj.id = id;
        end
        
        function shape = getShape(obj)
            shape = obj.shape;
        end 
        function id = getID(obj)
            id = obj.id;
        end
        function setShapeID(obj, shapeID)
            obj.shapeID = shapeID;
        end
        function shapeID = getShapeID(obj)
             shapeID = obj.shapeID;
        end
    end
end

