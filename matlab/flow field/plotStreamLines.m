function[] = plotStreamLines(Field,X,Y)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    PSI = zeros(size(X));
    U = zeros(size(X));
    V = zeros(size(X));
    
    [r,c] = size(Field);
    for i=1:1:r
        if Field(i,1) == 1
            [ui,vi,psii] = sourceFlow(Field(i,2),Field(i,3),Field(i,4),X,Y);
            PSI = PSI+psii;
            U = U + ui;
            V = V+vi;
        elseif Field(i,1) == 2
            [ui,vi,psii] = doubletFlow(Field(i,2),Field(i,3),Field(i,4),X,Y);
            PSI = PSI+psii;
            U = U + ui;
            V = V+vi;          
        end
        
        
    end
    
    figure()
    contour(X,Y,PSI,  -5:.2:5);  %0:.2:5
    hold on
    %quiver(X,Y,U,V,1);
    
end

