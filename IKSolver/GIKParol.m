close all
clear all

tic
robot = importrobot("PAROL6.urdf", 'DataFormat','column');
toc

a_1 = 110.5e-3;
a_2 = 23.42e-3;
a_3 = 180.0e-3;
a_4 = 43.5e-3;
a_5 = 176.35e-3;
a_6 = 62.8e-3;
a_7 = 45.25e-3;

q1 = [1.5708
    0.0002
   -0.0002
    1.5708
    0.7854
   -1.5708];
q1 = [0 0 0 0 0 0]';

q2 = [1.5708
    0.0002
   -0.0002
   -0.7854
   -0.7854
   -4.7124
];

ee = robot.BodyNames{end};

figure(1)
show(robot, q1);
axis([-500e-3 500e-3 -500e-3 500e-3 0 500e-3]);

figure(2)
show(robot, q2)
axis([-500e-3 500e-3 -500e-3 500e-3 0 500e-3]);

tic
gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'position', 'orientation', 'jointbounds'});
toc

% constraints
posTgt = constraintPositionTarget(ee);

% eigentlich: [a_2+a_5+a_6 0 a_1+a_3+a_4-a_7]; kinematic chain stops before
% gripper for some reason
posTgt.TargetPosition = [0 a_2+a_5 a_1+a_3+a_4];
posTgt.Weights = 1;

oriTgt = constraintOrientationTarget(ee);
oriTgt.TargetOrientation = eul2quat([0 pi/2 pi/2],"ZYX");
oriTgt.Weights = 1;

jointBounds = constraintJointBounds(robot);

initGuessConfig = [0 0 0 0 0 0]';

% tic
% [config, info] = gik(initGuessConfig , posTgt, oriTgt, jointBounds);
% toc

% show(robot, config);
% axis([-500e-3 500e-3 -500e-3 500e-3 0 500e-3]);