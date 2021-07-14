function  stream_display(PSI,X,Y)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


figure()
contour(X,Y,PSI, -1:.01:1 , 'ShowText', 'off');  %0:.2:5 -2*pi:.2:2*pi


end

