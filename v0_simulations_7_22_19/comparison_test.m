trials = 1000;
storeCounts = zeros(trials, 1);
for i = 1:trials
    temp = single_source_v0(false);
    storeCounts(i) = temp;
end
figure
histogram(storeCounts);
disp(mean(storeCounts));
disp(std(storeCounts));