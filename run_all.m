%-------Main Run File for Program
%build robot, generate data, train NN, benchmark, save artifacts
clear; clc; close all;
addpath(genpath('src'));
if ~exist('data','dir'), mkdir data; end
if ~exist('figures','dir'), mkdir figures; end

%robot model
[robot, L, limits] = makeRobot();

%generate big dataset
N = 100000;
fprintf('Generating dataset with N = %d...\n', N);
data = genDataset(robot, L, limits, N);

%train the neural network
fprintf('Training neural network...\n');
net = trainNN(data, L);
save('data/net.mat','net');

%benchmark data on sample of 1000 data points
fprintf('Benchmarking methods...\n');
n_bench = 1000; 
X_test_subset = data.test.X(1:n_bench, :);
Q_test_subset = data.test.Q(1:n_bench, :);
results = benchmark(robot, L, limits, net, X_test_subset, Q_test_subset);

%print headline data
fprintf('\n=== HEADLINE RESULTS ===\n');
fprintf('Median FK error (mm): Closed-form: %.3f | Numeric IK: %.3f | NN: %.3f\n', ...
    results.mm(1), results.mm(2), results.mm(3));
fprintf('Latency (s) for %d item batch: Closed-form: %.4f | Numeric IK: %.4f | NN: %.4f\n', ...
    n_bench, results.t(1), results.t(2), results.t(3));

%plots
figure; bar(results.t); xticklabels({'Closed-form','Numeric IK','NN'});
ylabel('Time (s) over test set'); title('Latency (timeit)'); grid on;
saveas(gcf,'figures/latency_bar.png');

figure; bar([results.mm(1) results.mm(2) results.mm(3)]);
xticklabels({'Closed-form','Numeric IK','NN'});
ylabel('Median FK error (mm)'); title('Accuracy (median over test)'); grid on;
saveas(gcf,'figures/accuracy_bar.png');

%save everything for app
save('data/artifacts.mat','robot','L','limits');
fprintf('\nDone. You can now run the programmatic app with:\n');
fprintf(' IKCompareApp\n');