function Q = predictNN(net, X)
X = double(X);
if isvector(X), X = reshape(X,1,[]); end
if isstruct(net) && isfield(net,'mode') && strcmp(net.mode,'hybrid_q1_rhoz_series')
    q1  = atan2(X(:,2), X(:,1));
    rho = hypot(X(:,1), X(:,2));
    z   = X(:,3);
    U   = single([rho z]);                  
    Y = predict(net.nn, U);                
    s2 = Y(:,1); s3 = Y(:,2);
    c2 = Y(:,3); c3 = Y(:,4);
    q2 = atan2(s2, c2);
    q3 = atan2(s3, c3);
    Q = [q1 q2 q3];
    Q = mod(Q + pi, 2*pi) - pi;
    return;
end

if isstruct(net) && isfield(net,'dlnet')
    Xn = (double(X) - net.mu) ./ net.sigma;
    dlX = dlarray(single(Xn'), 'CB');
    Y = gather(extractdata(predict(net.dlnet, dlX)))'; % N x 6
else
    Y = predict(net, single(X)); % SeriesNetwork path
end

s = Y(:,1:3); c = Y(:,4:6);
Q = atan2(s, c);
Q = double(mod(Q + pi, 2*pi) - pi);
end
