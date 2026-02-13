%genDataset for the solvers and AI:
% - sample q in joint space
% - compute X via rigidBodyTree FK
% - keep only samples where q1 matches atan2(y,x) (single yaw branch)
% - keep only samples with positive planar radius (avoid r<0 branch)
% - enforce q3 >= 0 (elbow-down)

function data = genDataset(robot, L, limits, N)
rho_min = 1e-3;      
yaw_tol = 1e-3;      
Q = zeros(N,3);
X = zeros(N,3);
k = 0;
batch = max(5000, 5*N);   
low3  = max(0, limits(3,1));
high3 = limits(3,2);

while k < N
    M = min(batch, (N-k)*50);
    q1 = limits(1,1) + rand(M,1) .* (limits(1,2) - limits(1,1));
    q2 = limits(2,1) + rand(M,1) .* (limits(2,2) - limits(2,1));
    if low3 < high3
        q3 = low3 + rand(M,1) .* (high3 - low3);
    else
        tmp = limits(3,1) + rand(M,1) .* (limits(3,2) - limits(3,1));
        q3  = abs(tmp);
    end
    q = [q1 q2 q3];
    q = mod(q + pi, 2*pi) - pi;
    r = L.L2*cos(q(:,2)) + L.L3*cos(q(:,2) + q(:,3));
    keep = (r >= rho_min);
    q = q(keep,:);
    if isempty(q), continue; end
    M2 = size(q,1);
    Xc = zeros(M2,3);
    for i = 1:M2
        T = getTransform(robot, q(i,:), 'tool');
        Xc(i,:) = tform2trvec(T);
    end
    rho = hypot(Xc(:,1), Xc(:,2));
    q1_atan = atan2(Xc(:,2), Xc(:,1));
    dq1 = wrapToPi(q1_atan - q(:,1));
    keep2 = (rho >= rho_min) & (abs(dq1) <= yaw_tol);
    q  = q(keep2,:);
    Xc = Xc(keep2,:);
    if isempty(q), continue; end
    take = min(size(q,1), N-k);
    Q(k+1:k+take,:) = q(1:take,:);
    X(k+1:k+take,:) = Xc(1:take,:);
    k = k + take;
end

nTrain = floor(0.8*N);
nVal   = floor(0.1*N);
idx = randperm(N);
itr = idx(1:nTrain);
ivl = idx(nTrain+1:nTrain+nVal);
its = idx(nTrain+nVal+1:end);
data.train.X = X(itr,:); data.train.Q = Q(itr,:);
data.val.X   = X(ivl,:); data.val.Q   = Q(ivl,:);
data.test.X  = X(its,:); data.test.Q  = Q(its,:);
end
