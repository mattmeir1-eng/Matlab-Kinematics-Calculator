function [degErr, mmErr] = metrics(robot, Q_true, Q_pred)

Q_true = double(Q_true);
Q_pred = double(Q_pred);
N = size(Q_true,1);
degErr = rad2deg(wrapToPiLocal(Q_pred - Q_true));
mmErr = zeros(N,1);
for i=1:N
pt = tform2trvec(getTransform(robot, Q_true(i,:), 'tool'));
pp = tform2trvec(getTransform(robot, Q_pred(i,:), 'tool'));
mmErr(i) = 1e3 * norm(pp - pt);
end
end

function A = wrapToPiLocal(A)
A = mod(A + pi, 2*pi) - pi;
end