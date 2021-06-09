function [PSI, PHI] = getPsiPhi(X,Y, Field)
[m,n] = size(Field);
PHI = zeros(size(X));
PSI = zeros(size(X));
for r = 1:1:m
    %base world flow
   if r == 1
       [Ui,Vi,PSIi,PHIi] = baseWorld(Field(r,1),Field(r,2), Field(r,3), Field(r,4),Field(r,5),X,Y);
       PSI = PSI+PSIi;
       PHI = PHI +PHIi;
     %obstacle flow
   else
       [Ui,Vi,PSIi,PHIi] = objectFlow(Field(r,1), Field(r,2), Field(r,3), Field(r,4), Field(r,5),X,Y);
       PSI = PSI+PSIi;
       PHI = PHI +PHIi;
   end
       
end
end

