close all
clear all

robot = importrobot("PAROL6.urdf");
robot.DataFormat = 'column';

show(robot, [0;0;0;0;0;0]);
axis([-500e-3 500e-3 -500e-3 500e-3 0 500e-3]);

ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 0.8 0.8 0.8]';
initGuessConfig = [0 0 0 0 0 0]';

poseTarget = se3([0 pi/2 pi/2], "eul", 'ZYX', [0 0.2 0.2]);
ee = robot.BodyNames{end};

tic
[config, info] = ik(ee, tform(poseTarget), weights, initGuessConfig);
toc

show(robot, config);
axis([-500e-3 500e-3 -500e-3 500e-3 0 500e-3]);
info