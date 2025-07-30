clear all
close all

load("robot_v2.mat");

a_1 = 110.5e-3;
a_2 = 23.42e-3;
a_3 = 180.0e-3;
a_4 = 43.5e-3;
a_5 = 176.35e-3;
a_6 = 62.8e-3;
a_7 = 45.25e-3;

figure(1)
rigidBodyTree_visualize(manipulator_rigidBodyTree, ...
                        [0;0;0;0;0;0], ...
                        axes_limits = [-500e-3 500e-3 -500e-3 500e-3 0 500e-3], ...
                        rigidBody_frame_to_turn_off = [2 4 6 8 10 12])

targetPos = [0e-3;
             (a_2+a_5+a_6);
             (a_1+a_3+a_4-a_7)]

targetPose = eye(4);

%rot = rotz(180)*roty(90);
%targetPose(1:3,1:3) = rot;

targetPose(1:3,4) = targetPos;

ik = inverseKinematics('RigidBodyTree', manipulator_rigidBodyTree)
weights = [1 1 1 0.8 0.8 0.8]';
initGuess = [0 -pi/2 pi 0 0 pi]';
%initGuess = [0 0 0 0 0 0]';

[configSol, solInfo] = ik('rigidBody_link_6', targetPose, weights, initGuess)

figure(2)
rigidBodyTree_visualize(manipulator_rigidBodyTree, ...
                        configSol, ...
                        axes_limits = [-500e-3 500e-3 -500e-3 500e-3 0 500e-3], ...
                        rigidBody_frame_to_turn_off = [2 4 6 8 10 12])