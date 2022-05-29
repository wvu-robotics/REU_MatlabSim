% simple boids simulation
boids_count=10;
boids=Boid.empty;
Ks = 1;
Ka = 1;
Kc = 100;
time = 10000;

for i=1:boids_count
   boids(i)=Boid(rand*640/3,rand*360/3, Ks,Ka, Kc,boids_count,1);
end



flock=Flock(boids,[640/3 360/3]);
f = figure;
plane = Plane(f,[610/3 360/3],boids);
flock.run(plane,time);

figure()
for b = 1:boids_count
    for t = 1:time
    quiver(boids(i).path(t,1), boids(i).path(t,2), boids(i).path(t,3), boids(i).path(t,4), 0);
    hold on;
    
    end
end