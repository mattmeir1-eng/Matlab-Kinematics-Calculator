%ik_numeric  wrapper around inverseKinematics with multiple initial seeds.
%returns best-position solution found (smallest |p(q)-tgt|).
function qbest = ik_numeric(robot, tgtXYZ, qseeds)
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestarts = true;
ik.SolverParameters.MaxIterations       = 1500;   
ik.SolverParameters.GradientTolerance   = 1e-12; 
ik.SolverParameters.SolutionTolerance   = 1e-6;  

weights = [0 0 0 1 1 1];
T = trvec2tform(tgtXYZ);
bestErr = inf; qbest = [];
for i = 1:size(qseeds,1)
    [q, solInfo] = ik('tool', T, weights, qseeds(i,:));
    if string(solInfo.Status) == "success"
        p = tform2trvec(getTransform(robot, q, 'tool'));
        err = norm(p - tgtXYZ);
        if err < bestErr, bestErr = err; qbest = q; end
    end
end
end
