function net = trainNN(data, L)

Xtr = single(data.train.X);  Qtr = single(data.train.Q);
Xval = single(data.val.X);   Qval = single(data.val.Q);
rho_tr = hypot(Xtr(:,1), Xtr(:,2));
z_tr   = Xtr(:,3);
Utr    = [rho_tr z_tr];
rho_val = hypot(Xval(:,1), Xval(:,2));
z_val   = Xval(:,3);
Uval    = [rho_val z_val];
Q23tr  = Qtr(:,2:3);
Q23val = Qval(:,2:3);

% targets: [sin(q2) sin(q3) cos(q2) cos(q3)]
Ttr  = single([sin(Q23tr)  cos(Q23tr)]);
Tval = single([sin(Q23val) cos(Q23val)]);

%3 layers: wide -> medium -> narrow
layers = [
    featureInputLayer(2,'Normalization','zscore','Name','in')
    fullyConnectedLayer(512,'Name','fc1')
    reluLayer('Name','r1')
    fullyConnectedLayer(256,'Name','fc2')
    reluLayer('Name','r2')
    fullyConnectedLayer(128,'Name','fc3')
    reluLayer('Name','r3')
    fullyConnectedLayer(4,'Name','head')
    regressionLayer('Name','reg')
];

opts = trainingOptions('adam', ...
    'MaxEpochs', 60, ...             
    'MiniBatchSize', 4096, ...       
    'InitialLearnRate', 0.005, ...   
    'LearnRateSchedule', 'piecewise', ... 
    'LearnRateDropFactor', 0.1, ...
    'LearnRateDropPeriod', 20, ...   
    'Shuffle','every-epoch', ...
    'ValidationData',{Uval, Tval}, ...
    'ValidationFrequency', 50, ...
    'ValidationPatience', 6, ...
    'Verbose', true, ...
    'Plots','training-progress');

fprintf('Training High-Volume Network (Batch 4096)...\n');
nn = trainNetwork(Utr, Ttr, layers, opts);
net.mode = 'hybrid_q1_rhoz_series';
net.nn   = nn;
net.L    = L;
f = @() predict(nn, Uval);
T = timeit(f);
fprintf('NN(q2,q3) inference: %.3f ms/sample\n', 1e3*T/size(Uval,1));
end