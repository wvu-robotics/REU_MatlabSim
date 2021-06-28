figure
pos1 = [0.1 0.3 0.3 0.3];
subplot('Position',pos1)
y = magic(4);
plot(y)
title('First Subplot')

pos2 = [0.5 0.15 0.4 0.7];
subplot('Position',pos2)
bar(y)
title('Second Subplot')