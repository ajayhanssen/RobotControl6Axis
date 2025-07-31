close all
clear all

tic
robot = importrobot("PAROL6.urdf", 'DataFormat','column');
toc

ee = robot.BodyNames{end};

show(robot, [0;0;0;0;0;0]);
axis([-500e-3 500e-3 -500e-3 500e-3 0 500e-3]);

tic
gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'position', 'orientation', 'jointbounds'});
toc

% constraints
posTgt = constraintPositionTarget(ee);
posTgt.TargetPosition = [0 0.2 0.2];
posTgt.Weights = 1;

oriTgt = constraintOrientationTarget(ee);
oriTgt.TargetOrientation = eul2quat([0 pi/2 pi/2],"ZYX");
oriTgt.Weights = 1;

jointBounds = constraintJointBounds(robot);

initGuessConfig = [0 0 0 0 0 0]';

tic
[config, info] = gik(initGuessConfig , posTgt, oriTgt, jointBounds);
toc

show(robot, config);
axis([-500e-3 500e-3 -500e-3 500e-3 0 500e-3]);