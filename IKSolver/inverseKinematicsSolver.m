clear all
close all

load("robot_v2.mat");

figure(1)
rigidBodyTree_visualize(manipulator_rigidBodyTree, ...
                        [0;0;0;0;0;0], ...
                        axes_limits = [-500e-3 500e-3 -500e-3 500e-3 0 500e-3], ...
                        rigidBody_frame_to_turn_off = [2 4 6 8 10 12])

targetPos = [150e-3;
             250e-3;
             200e-3]

targetPose = eye(4);
targetPose(1:3,4) = targetPos;

ik = inverseKinematics('RigidBodyTree', manipulator_rigidBodyTree)
weights = [1 1 1 0.8 0.8 0.8]';
initGuess = [0 -pi/2 pi 0 0 pi]';

[configSol, solInfo] = ik('rigidBody_link_6', targetPose, weights, initGuess)

figure(2)
rigidBodyTree_visualize(manipulator_rigidBodyTree, ...
                        configSol, ...
                        axes_limits = [-500e-3 500e-3 -500e-3 500e-3 0 500e-3], ...
                        rigidBody_frame_to_turn_off = [2 4 6 8 10 12])