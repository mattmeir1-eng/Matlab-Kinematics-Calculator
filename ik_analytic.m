%tries multiple sign/branch conventions and returns all candidates within limits.
function [Qlist, branches] = ik_analytic(xyz, L, limits)
x = xyz(1); y = xyz(2); z = xyz(3);

%base yaw
q1 = atan2(y,x);

%reduce to planar 2R in (r, z')
r  = hypot(x,y);
zp = z - L.L1;

%elbow angle
D  = (r^2 + zp^2 - L.L2^2 - L.L3^2) / (2*L.L2*L.L3);
if D < -1 || D > 1
    Qlist = zeros(0,3); branches = strings(0,1); return;
end
q3c = [atan2(+sqrt(max(0,1-D^2)), D), ...
       atan2(-sqrt(max(0,1-D^2)), D)];   

theta = atan2(zp, r);

%build candidate sets
cands = [];
bstrs = strings(0,1);
for j = 1:2
    q3 = q3c(j);
    phi = atan2(L.L3*sin(q3), L.L2 + L.L3*cos(q3));
    for s = [+1, -1]   
        q2 = theta + s*phi;
        basePairs = [q2, q3;
                    -q2, q3;
                     q2,-q3;
                    -q2,-q3];
        for k = 1:size(basePairs,1)
            q = [q1, basePairs(k,1), basePairs(k,2)];
            q = mod(q + pi, 2*pi) - pi;
            if all(q' >= limits(:,1) & q' <= limits(:,2))
                cands = [cands; q];
                bstrs = [bstrs; sprintf("branch%u s%+d flip%u", j, s, k)];
            end
        end
    end
end

Qlist    = cands;
branches = bstrs;
end
