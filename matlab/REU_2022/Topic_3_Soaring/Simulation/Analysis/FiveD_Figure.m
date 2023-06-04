clear
clc
T = readtable("2022-07-19 Data.xlsx","Sheet","Regular");
TM = T{:,:};
TMA = zeros(3360/5,17);
for row = 1:size(TM,1)/5
    TMA(row,:) = mean(TM(5*row-4:5*row,:),1);
end
TA = array2table(TMA,'VariableNames',T.Properties.VariableNames);

normedFT = TA.flightTime/288000;
color = hsv2rgb([normedFT,ones(672,1),ones(672,1)]);
normedSurvivors = 2+TA.surviving;
hold on
for p = 1:672
    plot3(TA.heightScore(p),TA.explorationPercent(p),TA.thermalUseScore(p),'.','Color',color(p,:),'MarkerSize',normedSurvivors(p));
end
xlabel("Height Score");
ylabel("Exploration Percent");
zlabel("Thermal Use Score");
legend("Color is Flight Time, Size is # of Survivors")
ax = gca;
ax.Color = [0.2 0.2 0.2];
view(3)