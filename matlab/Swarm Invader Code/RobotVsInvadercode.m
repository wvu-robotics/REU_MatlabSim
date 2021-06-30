n=1; %robot
a=1; %invader
curr_x = 1;
curr_y = 0.8;
invader_x = 0.2;
invader_y = 0.2;
radius = 0.5;

% while 1
%         within_radius(curr_x, curr_y, invader_x, invader_y, radius);
%         distance = sqrt((curr_x-invader_x).^2 + (curr_y - invader_y).^2);
%         if (distance <= radius)
%            % return true %robot stop moving
%             break;
%         end
%         if (distance > radius)
%            % return false %continue moving
%            disp("it is false")
% %            break;
%         end 
%      fprintf('Starting Script:')
%      if(1 == within_radius(curr_x, curr_y, invader_x, invader_y, radius));
%       disp("IT IS True")
%       
% %      elseif
% %      print("IT IS False")
% %  
%         end   
% end
function result= within_radius(curr_x, curr_y, invader_x, invader_y, radius);
result = 1;
end
while 1
    x = rand (0,1);
    if (x < obj.ag)
        be_violent (aggressive)
    end
    if (x > obj.ag)
        be_peaceful(runaway)
    end
end

