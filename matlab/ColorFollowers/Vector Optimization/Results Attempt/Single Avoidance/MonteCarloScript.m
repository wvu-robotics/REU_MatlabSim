clear
clc
close all

iterations = 1000;
paramRows = 4;
paramCols = 3;
display = false;
costs = zeros(iterations, 1);
for i = 1:iterations
parameters = (rand(paramRows, paramCols) - 0.5) * 2;
costs(i) = SingleAvoidOptimizeable(parameters, display);
disp(i);
end
% for i = 1:iterations
%    close(figure(i)); 
% end
histogram(costs(costs<1000), 'BinMethod', 'sqrt')

