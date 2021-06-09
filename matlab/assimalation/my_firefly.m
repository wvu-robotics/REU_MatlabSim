% my firefly syncronization

numbots = 1000;

env = zeros(100,100,3);
for r = 1:numbots
    x = round(80*rand(1,1)) + 10;
    y = round(80*rand(1,1)) + 10;
    env(x,y,1) = 1;
    env(x,y,2) = rand(1,1);
end

subplot(1,3,1)
imshow(env(:,:,1));
title("robots")

subplot(1,3,2)
imshow(env(:,:,2));
title("charge")

subplot(1,3,3)
imshow(env(:,:,3))
title("flash")

for t = 1:1000000
    
    for x = 1:100
        for y = 1:100
            if env(x,y,1) == 1
                
                FORCE = [0,0];
                %search local enviorment for flashes
                for i = x-5:x+5
                    for j = y-5:y+5
                        if env(i,j,3) == 1
                            env(x,y,2) = env(x,y,2) + .01;
                            
                            if env(x,y,3) == 1 && i ~= x && j ~= y
                                dx= i-x;
                                dy =j-y;
                                d = norm([dx,dy]);
                               FORCE = FORCE + [ (dx/(d*d)) , (dy/(d*d)) ]; 
                            end
                            
                        end
                        
                            
                    end
                end
                
                env(x,y,2) = env(x,y,2) + .05;
                
                % random walk the agent + influence
                totF = norm(FORCE);
                if totF == 0
                    totF = 1;
                end
                dx = round(FORCE(1)); %2*rand(1,1)-1 + /totF
                dy = round(FORCE(2)); %2*rand(1,1)-1 + /totF
                
                if dx + x > 10 && dx + x < 90
                    
                    if dy + y > 10 && dy + y < 90
                        if env(x+dx, y+dy,1) == 0
                            env(x+dx,y+dy,1)= 1;
                            env(x+dx, y+dy,2) = env(x,y,2);
                            env(x+dx, y+dy,3) = env(x,y,3);
                            env(x,y,1) = 0;
                            env(x,y,2) = 0;
                            env(x,y,3) = 0;
                        end
                    end
                end
               
                 
            end
        end
    end
   
   env(:,:,3) = env(:,:,2) >= .9; %.9
   
   charges = env(:,:,2);
   ind = find(charges >= 1);
   charges(ind) = 0;
   env(:,:,2) = charges;
    
    clf();
    
    subplot(1,3,1)
    imagesc(env(:,:,1));
    title("robots")

    subplot(1,3,2)
    imagesc(env(:,:,2));
    title("charge")
    
    subplot(1,3,3)
    imagesc(env(:,:,3))
    title("flash")
    
    set(gcf,'WindowState','fullscreen')
    
    pause(.00001)
end