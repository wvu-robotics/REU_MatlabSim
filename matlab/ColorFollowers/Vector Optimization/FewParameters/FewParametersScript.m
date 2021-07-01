clear
close all;
clc

%% Setting up GUI
global params;
params = zeros(5,1);
f = figure('Units','inches','Position',[2,2,7.5,6]);
paramPanel = uipanel(f,'Position',[0.05 0.05 0.9 0.3]);
figPanel = uipanel(f,'Position',[0.05,.3,.6,.65]);
dispPanel = uipanel(f,'Position',[0.65,.3,.3,.65]);
figAxis = axes(figPanel);
Slider1 = uicontrol(paramPanel, 'style','slide',...
    'unit','normalized',...
    'position',[.05 .05 .4 .1],...
    'min',-5,'max',5,'val',(rand()-0.5)*10);
Slider2 = uicontrol(paramPanel, 'style','slide',...
    'unit','normalized',...
    'position',[.05 .3 .4 .1],...
    'min',-5,'max',5,'val',(rand()-0.5)*10);
Slider3 = uicontrol(paramPanel, 'style','slide',...
    'unit','normalized',...
    'position',[.05 .55 .4 .1],...
    'min',-5,'max',5,'val',(rand()-0.5)*10);
Slider4 = uicontrol(paramPanel, 'style','slide',...
    'unit','normalized',...
    'position',[.5 .05 .4 .1],...
    'min',-5,'max',5,'val',(rand()-0.5)*10);
Slider5 = uicontrol(paramPanel, 'style','slide',...
    'unit','normalized',...
    'position',[.5 .3 .4 .1],...
    'min',-5,'max',5,'val',(rand()-0.5)*10);
text1 = uicontrol(paramPanel, 'style','text',...
    'unit','normalized',...
    'position',[.05 .15 .4 .1],'string','Color Range -> Centroid');
text2 = uicontrol(paramPanel, 'style','text',...
    'unit','normalized',...
    'position',[.05 .4 .4 .1],'string','Density -> Centroid');
text3 = uicontrol(paramPanel, 'style','text',...
    'unit','normalized',...
    'position',[.05 .65 .4 .1],'string','Color Range -> OrthoToCentroid');
text4 = uicontrol(paramPanel, 'style','text',...
    'unit','normalized',...
    'position',[.5 .15 .4 .1],'string','Density -> OrthoToCentroid');
text5 = uicontrol(paramPanel, 'style','text',...
    'unit','normalized',...
    'position',[.5 .4 .4 .1],'string','Angle for Density Weighting');
agent1Text = uicontrol(dispPanel, 'style','text',...
    'unit','normalized',...
    'position',[.02 .85 .2 .1],'string','Agent 1:');
agent1ColorRange = uicontrol(dispPanel, 'style','edit',...
    'unit','normalized',...
    'position',[.25 .9 .2 .05]);
agent1Density = uicontrol(dispPanel, 'style','edit',...
    'unit','normalized',...
    'position',[.5 .9 .2 .05]);
agent2Text = uicontrol(dispPanel, 'style','text',...
    'unit','normalized',...
    'position',[.02 .75 .2 .1],'string','Agent 2:');
agent2ColorRange = uicontrol(dispPanel, 'style','edit',...
    'unit','normalized',...
    'position',[.25 .8 .2 .05]);
agent2Density = uicontrol(dispPanel, 'style','edit',...
    'unit','normalized',...
    'position',[.5 .8 .2 .05]);

% for i = 1:100
%    params(1) = Slider1.Value;
%    params(2) = Slider2.Value;
%    params(3) = Slider3.Value;
%    params(4) = Slider4.Value;
%    params(5) = Slider5.Value;
%    disp(params);
%    pause(.05);
% end



%% Setting Up Sim
%   World Building
numberOfAgents = 20;
agentRadius = .5;
timeStep = .1;
mapSize = 10;
counter = 0;
maxSpeed = 2;
sensingRange = 8;
connectionRange = 2;
newParameters = zeros(5,1);
display = true;
timeSteps = 500;

spawnType = 'random';
%spawnType = 'opposingGroups';

%Set f to appropriate color handle
f = @FewParameterController;

%Set up the environment
ENV = agentEnv(numberOfAgents,agentRadius,mapSize,timeStep); 

%Spawn the agents using a custom spawn function
[initPositions, goalLocations] = FewParamSpawn(numberOfAgents, spawnType, mapSize);

%set some variables in the environment
ENV.setAgentPositions(initPositions);
ENV.setGoalPositions(goalLocations);
ENV.setAgentVelocities(zeros(numberOfAgents,2));
ENV.realTime = false;
ENV.pathVisibility(false)

%Set some variables for each agent
for i = 1:numberOfAgents
    ENV.agents(i).setController(f);
    ENV.agents(i).createProperty('parameters',newParameters);
    ENV.agents(i).createProperty('colorRange',0);
    ENV.agents(i).createProperty('density',0);
    ENV.agents(i).measuringRange = sensingRange;
    ENV.agents(i).maxSpeed = maxSpeed;
end 

%% Running Sim
cost = 0;
while(true)
   params(1) = Slider1.Value;
   params(2) = Slider2.Value;
   params(3) = Slider3.Value;
   params(4) = Slider4.Value;
   params(5) = Slider5.Value;
   set(agent1ColorRange, 'String', num2str(ENV.agents(1).getProperty('colorRange')));
   set(agent1Density, 'String', num2str(ENV.agents(1).getProperty('density')));   
   set(agent2ColorRange, 'String', num2str(ENV.agents(2).getProperty('colorRange')));
   set(agent2Density, 'String', num2str(ENV.agents(2).getProperty('density')));   
   
   
   
   for i = 1:length(ENV.agents)
      ENV.agents(i).setProperty('parameters',params);
   end
    customTick(ENV, timeStep, display, f, figAxis, mapSize);
    counter = counter + 1;
    fprintf("Time: %i \n",counter)
    if counter > timeSteps
       break 
    end
end

%% Required Functions
    function customTick(ENV, timeStep, display, f, figAxis, mapSize)
            for ii = 1:length(ENV.agents)
                ENV.agents(ii).callMeasurement(ENV);
                ENV.agents(ii).callController;     
                customPhys(ENV, ii, timeStep);
            end
            if display
                customDraw(ENV, f, figAxis, mapSize);
            end
    end
    function customDraw(ENV, f, figAxis, mapSize)
        figure(1);
        cla(figAxis)
            hold on
            xlim([-mapSize,mapSize]);
            ylim([-mapSize,mapSize]);
            for ii = 1:length(ENV.agents)
                
                RGB = ENV.agents(ii).color;
                %RGB = RGB/256;
                plot(ENV.agents(ii).pose(1), ENV.agents(ii).pose(2), '.', 'MarkerEdge', RGB, 'MarkerSize', 25);
                set(gca,'Color','k');
            end
            hold off
            pause(0.01)
    end
    function customPhys(ENV,id, timeStep)
        controlVel = ENV.agents(id).velocityControl;
        ENV.agents(id).velocity = controlVel;
        ENV.agents(id).pose = ENV.agents(id).pose + ENV.agents(id).velocity*timeStep;
    end
    
