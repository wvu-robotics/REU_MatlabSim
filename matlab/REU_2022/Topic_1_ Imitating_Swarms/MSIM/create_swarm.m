function AGENTS = create_swarm(Nagents,mapsize,range)

mapsize=mapsize-5;
AGENTS = agent.empty();
gains = [3,3,0.5];
Ka = gains(1); % Alingment gain
Ks = gains(2); % Separation gain
Kc = gains(3); % Cohesion gain

for i=1:Nagents
   x = randi([-mapsize mapsize]);
   y = randi([-mapsize mapsize]);
   AGENTS(i)=agent(x,y,Ks,Ka,Kc,i);
   AGENTS(i).range = range;
   AGENTS(i).ID = i;
   
   AGENTS(i).max_speed = .5;
end

end