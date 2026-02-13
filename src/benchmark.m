% benchmark compare closed-form, numerical IK, and NN on times and accuracy.
function results = benchmark(robot, L, limits, net, Xtest, Qtest)
N = size(Xtest,1);

%closed-form: loop over all test targets
f_cf = @() cfSolveAll(Xtest, L, limits, robot);
t_cf = timeit(f_cf);

%numerical IK: try several random seeds
q0s = (rand(5,3)-0.5).*repmat([pi, pi/2, pi/2],5,1);
f_num = @() numSolveAll(Xtest, robot, q0s, L, limits);
t_num = timeit(f_num);

%NN: vectorized predict
f_nn = @() predictNN(net, Xtest);
t_nn = timeit(f_nn);

%accuracy against known joint values
Q_cf = cfSolveAll(Xtest, L, limits, robot);
Q_num = numSolveAll(Xtest, robot, q0s, L, limits);
Q_nn = predictNN(net, Xtest);
[deg_cf, mm_cf] = metrics(robot, Qtest, Q_cf);
[deg_num, mm_num] = metrics(robot, Qtest, Q_num);
[deg_nn, mm_nn] = metrics(robot, Qtest, Q_nn);
results.t = [t_cf, t_num, t_nn];
results.mm = [median(mm_cf), median(mm_num), median(mm_nn)];
results.deg = [median(abs(deg_cf))', median(abs(deg_num))', median(abs(deg_nn))'];
end

function Q = cfSolveAll(X, L, limits, robot)
N = size(X,1); Q = zeros(N,3);
for i=1:N
[Qs, ~] = ik_analytic(X(i,:), L, limits);
if isempty(Qs), Q(i,:) = [NaN NaN NaN]; continue; end
bestErr = inf; bestQ = Qs(1,:);
for k=1:size(Qs,1)
p = tform2trvec(getTransform(robot, Qs(k,:), 'tool'));
e = norm(p - X(i,:));
if e < bestErr, bestErr = e; bestQ = Qs(k,:); end
end
Q(i,:) = bestQ;
end
end

function Q = numSolveAll(X, robot, q0s, L, limits)
N = size(X,1); Q = zeros(N,3);
for i = 1:N
    [Qa,~] = ik_analytic(X(i,:), L, limits);
    if isempty(Qa)
        seeds = q0s;
    else
        seeds = [q0s; Qa];  
    end
    q = ik_numeric(robot, X(i,:), seeds);
    if isempty(q)
        Q(i,:) = [NaN NaN NaN];
    else
        Q(i,:) = q;
    end
end
end