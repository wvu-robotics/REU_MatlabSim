clear all;
% clc;
% C = randi([0,2],10,10);
% 
% map1 = [1,0,0;
%     0,1,0;
%     0,0,1];
% 
% colormap(map1);
% pcolor(C);


% s = struct('draw_val', 0, 'food', 0, 'robot', 0, 'obstacle' , 1);
% %Everytime a cell is modified it's draw_val must be updated
% 
% arr = repmat(s,100,100); %make a 10x10 array of structs
% 
% out = arr(:,:).food;


A(:,:,2:5) = randi([1,10],100,100,4);
A(:,:,1) = zeros();

x = sum(A,3); %squash the 3d matrix into a 2D one

for i=1:10000
    
    A(:,:,2:5) = randi([1,10],100,100,4);
    x = sum(A,3); %squash the 3d matrix into a 2D one

    pcolor(x);
    pause(0.01);
    
end

%disp(s.robot);