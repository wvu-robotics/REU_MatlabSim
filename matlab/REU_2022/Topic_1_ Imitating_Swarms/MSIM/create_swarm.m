function AGENTS = create_swarm(Nagents,map_length,range)

AGENTS = agent.empty();
gains = [10,5,10];
Ka = gains(1); % Alingment gain
Ks = gains(2); % Separation gain
Kc = gains(3); % Cohesion gain

for i=1:Nagents
   x = 2*rand*map_length - map_length;
   y = 2*rand*map_length - map_length;
   AGENTS(i)=agent(x,y,Ks,Ka,Kc,i);
   AGENTS(i).range = range;
   AGENTS(i).ID = i;
   
   AGENTS(i).max_speed = .5;
end

end