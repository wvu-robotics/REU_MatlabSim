% sine wave movement
t= 1:.1:10;
X = [.1,.2,.3,.4,.5];
Y = [0,0,0,0,0];

U = [];
V = [];

figure()
plot(X,Y);
for t = 1:.1:10
   U = abs(sin(t + X));
   X = X+U*.1;
   clf()
   plot(X, Y, '*');
   axis([0,5.5,-.5,.5]);
   pause(.01);
end











