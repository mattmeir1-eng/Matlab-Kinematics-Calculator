function p = fk_point(robot, q)
T = getTransform(robot, q, 'tool');
p = tform2trvec(T);
end