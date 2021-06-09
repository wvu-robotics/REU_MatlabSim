function [PSI, PHI] = plotObAvFlow(Field, X, Y)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

[PSI,PHI] = getPsiPhi(X,Y, Field);


figure()
contour(X,Y,PSI, -2*pi:.2:2*pi,'ShowText', 'on');  %0:.2:5
hold on
contour(X,Y,PHI, -2*pi:.2:2*pi, 'ShowText', 'on');

hold off



end

