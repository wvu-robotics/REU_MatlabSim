% color particles 

numbots = 1000; %number of robots in simulation

env = zeros(100,100,5); % [robot, red, green, blue, total particles]

%randomly spawn robots in the enviorment
for r = 1:numbots
    x = round(80*rand(1,1)) + 10;
    y = round(80*rand(1,1)) + 10;
    env(x,y,1) = 1;
    
end

center = 50; %center of map aka location of base (50,50)

for t = 1:1000000
    
     %pick 1000 random agents to trade
    for a = 1:1000  
        x = round(99*rand(1,1))+1; %x position of agent
        y = round(99*rand(1,1))+1; %y position of agent
        
            if env(x,y,1) == 1 %if the agent is a robot
         
                %search local enviorment for other robots
                for i = x-5:x+5
                    for j = y-5:y+5
                        %if we see another agent transfer a color particle based on weighted
                        %probability
                        if env(i,j,1) == 1
                            %calculate probability weights
                            W = [env(i,j,2),env(i,j,3), env(i,j,4)]./ sum([env(i,j,2),env(i,j,3), env(i,j,4)]);
                            
                            %check to make sure the other agent has a
                            %particle
                            if sum(W) > 0
                                color = randsample(1:3,1,true,W)+1; %pick color particle
                                env(x,y,color) = env(x,y,color)+1; %recieve particle
                                env(i,j,color) = env(i,j,color)-1; %remove particle from other agent
                            end
 
                        end
                        
                        %see the base and based on angle get two of the same
                        %type of particle
                        if sqrt((x - center)^2 + (y-center)^2) < 7.1 %within 5 squares
                           theta = atan2d(y-center,x-center);
                           if theta < -120 %red range
                               env(x,y,2) = env(x,y,2) + 2;
                           elseif theta < 0 %green
                               env(x,y,3) = env(x,y,3) + 2;
                           elseif theta < 120 %blue
                               env(x,y,4) = env(x,y,4) + 2;
                           else
                               env(x,y,2) = env(x,y,2) + 2;
                           end
                        end
                            
                        
                    end
                end
                
                %calculate the total number of particles
                env(x,y,5) = sum(env(x,y,2:4));

                % random walk the agent
                 dx = round((2*rand(1,1)-1));
                 dy = round((2*rand(1,1)-1)); 
  
                if dx + x > 10 && dx + x < 90
                    
                    if dy + y > 10 && dy + y < 90
                        if env(x+dx, y+dy,1) == 0
                            env(x+dx,y+dy,1)= 1;
                            env(x+dx, y+dy,2) = env(x,y,2);
                            env(x+dx, y+dy,3) = env(x,y,3);
                            env(x+dx, y+dy,4) = env(x,y,4);
                            env(x,y,1) = 0;
                            env(x,y,2) = 0;
                            env(x,y,3) = 0;
                            env(x,y,4) = 0;
                        end
                    end
                end 
            end
    end
 
  
   clf();
   
   %--------------------------uncomment to see individual planes
%    subplot(3,2,1)
%     imagesc(env(:,:,1));
%    subplot(3,2,2)
%     imagesc(env(:,:,2));
%    subplot(3,2,3)
%     imagesc(env(:,:,3));
%    subplot(3,2,4);
%     imagesc(env(:,:,4));
%    subplot(3,2,5)
  

   image = zeros(100,100,3);
   image(:,:,1) = env(:,:,2) ./ env(:,:,5);
   image(:,:,2) = env(:,:,3) ./ env(:,:,5);
   image(:,:,3) = env(:,:,4) ./ env(:,:,5);
    imshow(image);
   
    
    pause(.00001)
end



