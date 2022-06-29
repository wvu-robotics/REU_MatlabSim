clc;
clear;

numTrials = 10000000;
randNums = rand(1,numTrials);
absVal = zeros(1,numTrials);
bruteVal = zeros(1,numTrials);
threshold = 0.5;

%% Abs Method
c1 = clock();
for i=1:numTrials
    current = randNums(i);
    result = abs(current) > threshold;
    absVal(i) = result;
end
c2 = clock();
absTime = c2(6) - c1(6);
if(c2(5) ~= c1(5))
    absTime = absTime + 60;
end

%% BruteForce Method
c1 = clock();
for i=1:numTrials
    current = randNums(i);
    result = current > threshold || current < -threshold;
    bruteVal(i) = result;
end
c2 = clock();
bruteTime = c2(6) - c1(6);
if(c2(5) ~= c1(5))
    bruteTime = bruteTime + 60;
end

%% Check Equality
equal = true;
for i=1:numTrials
    if(absVal(i) ~= bruteVal(i))
        equal = false;
        fprintf("%g ~= %g\n",absVal(i),bruteVal(i));
    end
end

%% Print Results
fprintf("Equality: %g\n",equal);
fprintf("Trials Run: %g\n",numTrials);
fprintf("Abs Time: %g\n",absTime);
fprintf("BruteForce Time: %g\n",bruteTime);