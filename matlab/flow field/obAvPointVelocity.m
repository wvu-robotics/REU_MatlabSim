function [u,v] = obAvPointVelocity(xi,yi,Field)
%xi/yi point in flow, Field list of obstacles, X all x points, Y all y
%points and returns the x an y velocity at the given point
%   uses flow field potential and stream function equations to find the
%   velocity at a given point

[i,c] = size(Field);

u = 0;
v = 0;

for r = 1:1:i
    
    %source / sink flow
    if r == 1
     [ui,vi,psii,phii] = baseWorld(Field(r,1),Field(r,2), Field(r,3),Field(r,4), Field(r,5), xi,yi);
     u = u+ui;
     v = v+vi;
    % Obstacle Flow   
    else
       
       uj = 0;
       vj = 0;
       xj = Field(r,1);
       yj = Field(r,2);
       for j = [1:1:r-1, r+1:1:i]
            %source / sink flow
            if r == 1
             [ui,vi,psii,phii] = baseWorld(Field(r,1),Field(r,2), Field(r,3),Field(r,4), Field(r,5), xi,yi);
             u = u+ui;
             v = v+vi;
            % Obstacle Flow   
            elseif Field(r,1) == 0
                [uji,vji,psi,phij] = objectFlow(Field(j,1), Field(j,2),uj,vj, Field(j,5), xj,yj);
                uj = uj+uji;
                vj = vj+vji;
            end
       end
       [ui,vi,psii,phii] = objectFlow(Field(r,1), Field(r,2),uj,vj, Field(r,5), xi,yi);
       u = u +ui;
       v = v+vi;
        
    end
    
    



end

