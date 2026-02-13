%makeRobot build a simple 3R manipulator with base yaw and two pitch joints.
function [robot, L, limits] = makeRobot()
L = struct('L1',0.12,'L2',0.20,'L3',0.18);
limits = [deg2rad([-180 180]); 
deg2rad([-110 110]); 
deg2rad([-135 135])]; 

robot = rigidBodyTree('DataFormat','row','MaxNumBodies',3);

%base link1 yaw about z
body1 = rigidBody('link1');
j1 = rigidBodyJoint('j1','revolute');
j1.JointAxis = [0 0 1];
j1.PositionLimits = limits(1,:);
setFixedTransform(j1, eye(4));
body1.Joint = j1; addBody(robot, body1, robot.BaseName);


%yaw to link2 arm pitch about y
body2 = rigidBody('link2');
j2 = rigidBodyJoint('j2','revolute');
j2.JointAxis = [0 1 0];
j2.PositionLimits = limits(2,:);
T12 = trvec2tform([0 0 L.L1]);
setFixedTransform(j2, T12);
body2.Joint = j2; addBody(robot, body2, 'link1');


%link2 to link3 pitch about y
body3 = rigidBody('link3');
j3 = rigidBodyJoint('j3','revolute');
j3.JointAxis = [0 1 0];
j3.PositionLimits = limits(3,:);
T23 = trvec2tform([L.L2 0 0]);
setFixedTransform(j3, T23);
body3.Joint = j3; addBody(robot, body3, 'link2');


%tip of the robot
bodyE = rigidBody('tool');
jE = rigidBodyJoint('fixTool','fixed');
T3E = trvec2tform([L.L3 0 0]);
setFixedTransform(jE, T3E);
bodyE.Joint = jE; addBody(robot, bodyE, 'link3');
end