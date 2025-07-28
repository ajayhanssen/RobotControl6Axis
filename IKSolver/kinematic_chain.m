%% Clean Up
clc
clear all
close all

%% Command Window Options
format compact

%% General Link Transformation Matrix
% Symbolic DH-Parameters
syms phi d a alpha

% Pure Rotation Around z-Axes
T_Rz(phi) = [   cos(phi)   -sin(phi)    0 0;
                sin(phi)    cos(phi)    0 0;
                0           0           1 0;
                0           0           0 1];

% Pure Translation Along z-Axes
T_Tz(d) = [ 1 0 0 0;
            0 1 0 0;
            0 0 1 d;
            0 0 0 1];

% Pure Translation Along x-Axes
T_Tx(a)= [ 1 0 0 a;
           0 1 0 0;
           0 0 1 0;
           0 0 0 1];

% Pure Rotation Around x-Axes
T_Rx(alpha) = [ 1 0             0           0;
                0 cos(alpha)   -sin(alpha)  0;
                0 sin(alpha)    cos(alpha)  0;
                0 0             0           1];

% General Link Transformation Matrix
A_j_minus_1_j(phi, d, a, alpha) = ...
    T_Rz(phi) * T_Tz(d) * T_Tx(a) * T_Rx(alpha);

% Clean Up
clearvars -except A_j_minus_1_j phi d a alpha

%% Specific Link Transformation Matrices
% Joint Variables in rad, rad, and m
syms q_1 q_2 q_3 q_4 q_5 q_6

a_1 = 110.5e-3;
a_2 = 23.42e-3;
a_3 = 180.0e-3;
a_4 = 43.5e-3;
a_5 = 176.35e-3;
a_6 = 62.8e-3;
a_7 = 45.25e-3;

% DH-Paramters
% Order: [phi d a alpha]
% [ q_1       a_1  a_2  -pi/2 ;
%   q_2-pi/2  0    a_3   pi    ;
%   q_3+pi    0   -a_4   pi/2  ;
%   q_4      -a_5  0    -pi/2  ;
%   q_5       0    0     pi/2  ;
%   q_6+pi   -a_6 -a_7   pi]

% Specific Link Transformation Matrices
A_0_1(q_1) = A_j_minus_1_j(q_1, a_1, a_2, -pi/2);
A_1_2(q_2) = A_j_minus_1_j(q_2-pi/2, 0, a_3, pi);
A_2_3(q_3) = A_j_minus_1_j(q_3+pi, 0, -a_4, pi/2);
A_3_4(q_4) = A_j_minus_1_j(q_4, -a_5, 0, -pi/2);
A_4_5(q_5) = A_j_minus_1_j(q_5, 0, 0, pi/2);
A_5_6(q_6) = A_j_minus_1_j(q_6+pi, -a_6, -a_7, pi);

% Clean Up
clearvars -except A_0_1 A_1_2 A_2_3 A_3_4 A_4_5 A_5_6 q_1 q_2 q_3 q_4 q_5 q_6 a_1 a_2 a_3 a_4 a_5 a_6 a_7;

%% Create Overall Homogeneous Transformation Matrix
T_0_3(q_1, q_2, q_3, q_4, q_5, q_6) = ...
    A_0_1(q_1) * A_1_2(q_2) * A_2_3(q_3) * A_3_4(q_4) * A_4_5(q_5) * A_5_6(q_6);

% Clean Up
clearvars -except T_0_3 q_1 q_2 q_3 q_4 q_5 q_6 a_1 a_2 a_3 a_4 a_5 a_6 a_7

%% Forward Kinematics
% Axes Variable Vectors
% Use Zero Position for Testing Kinematical Model
deg = pi/180;

q_zero = [  0;
            0;
            0;
            0;
            0;
            0];

q_target_01 = [ 30*deg;
                30*deg;
                10*deg;
                20*deg;
                90*deg;
                45*deg];

% Relative Pose of {Endeffector} w.r.t. {base}
T_base_endeffector = forward_kinematics(T_0_3, q_zero)

%% Create rigidBodyTree Object Representation of Manipulator

a_1 = 110.5e-3;
a_2 = 23.42e-3;
a_3 = 180.0e-3;
a_4 = 43.5e-3;
a_5 = 176.35e-3;
a_6 = 62.8e-3;
a_7 = 45.25e-3;

% DH-Paramters
% Order: [phi d a alpha]
% [ q_1       a_1  a_2   -pi/2 ;
%   q_2-pi/2  0    a_3   pi    ;
%   q_3+pi    0    -a_4  pi/2  ;
%   q_4       -a_5 0     -pi/2 ;
%   q_5       0    0     pi/2  ;
%   q_6+pi    -a_6 -a_7  pi]
%

% Parameters With Seperated Joint Offsets
% [ (0)     0       0       0;
%   (q_1)   0.75   -0.6     pi;
%   (0)     0       0       0;
%   (q_2)   0.15   -0.4     0;
%   0       (0.30)  0       0;
%   pi      (q_3)   0       0]

parameters = [0     a_1      a_2       0    ;   % Offset 1
              0     0    0    -pi/2 ;   % Revolute joint 1

              -pi/2 0      0       0    ;   % Offset 2
              0     0      a_3     pi   ;   % Revolute joint 2

              pi    0      0       0    ;   % Offset 3
              0     0     -a_4     pi/2 ;   % Revolute joint 3

              0    -a_5    0       0    ;   % Offset 4
              0     0      0      -pi/2 ;   % Revolute joint 4

              0     0      0       0    ;   % Offset 5
              0     0      0       pi/2 ;   % Revolute joint 5

              pi    0      0       0    ;   % Offset 6
              0    -a_6   -a_7     pi  ];  % Revolute joint 6

                
% Create rigidBodyTree Object
manipulator_rigidBodyTree = create_manipulator(parameters);

%% Visualize rigidBodyTree Object
figure()

%show(manipulator_rigidBodyTree, q_zero)

rigidBodyTree_visualize(manipulator_rigidBodyTree, ...
                        [0;0;0;0;0;0], ...
                        axes_limits = [-500e-3 500e-3 -500e-3 500e-3 0 500e-3], ...
                        rigidBody_frame_to_turn_off = [2 4 6 8 10 12])

% Clean Up
clearvars parameters

%% Visualize rigidBodyTree Object
figure()

%show(manipulator_rigidBodyTree, q_zero)

rigidBodyTree_visualize(manipulator_rigidBodyTree, ...
                        q_zero, ...
                        axes_limits = [-500e-3 500e-3 -500e-3 500e-3 0 500e-3], ...
                        rigidBody_frame_to_turn_off = [2 4 6])

%% Define Start And Goal Endeffector Position
end_effector_position_start = [ -0.35;
                                 0.35;
                                 0];

end_effector_position_goal = [  0.35;
                                0.35;
                                0];

%% Joint-Space Motion
% Transform Start And Goal Endeffector Position Into Joint-Space

% You have to write your own inverse_kinematics function
q_start = ...
    inverse_kinematics(T_0_3, ...
    end_effector_position_start, ...
    'cfg 1');

q_goal = ...
    inverse_kinematics(T_0_3, ...
    end_effector_position_goal, ...
    'cfg 1');

% % Visualize Joint Variable Vector
% rigidBodyTree_visualize(manipulator_rigidBodyTree, ...
%                         q_start, ...
%                         axes_limits = [-0.75 0.75 -0.75 0.75 0 0.75], ...
%                         rigidBody_frame_to_turn_off = [2 4 6])
% 
% figure()
% rigidBodyTree_visualize(manipulator_rigidBodyTree, ...
%                         q_goal, ...
%                         axes_limits = [-0.75 0.75 -0.75 0.75 0 0.75], ...
%                         rigidBody_frame_to_turn_off = [2 4 6])

% Define Waypoints respectively Start and Goal Joint-Variable Vectors
q_points = [q_start(1) q_goal(1);
            q_start(2) q_goal(2);
            q_start(3) q_goal(3)];

% Duration of Motion in s
t_duration = 5;

% Start and End Time of Trajectory in s
t_points = [0 t_duration];

% Sample Time in s
t_sample = 0.1;

% Vector with Time Steps in s
t_vec = 0:t_sample:t_duration;

% Generate Cubic Joint-Space Trajectory
[q_traj, qd, qdd, ~] = cubicpolytraj(q_points, t_points, t_vec);

% Visualize Joint-Space Trajectory
figure()
plot_joint_space_trajectory(t_vec, q_traj)

% Visualize Manipulator Movement
figure()
visualize_manipulator(manipulator_rigidBodyTree, ...
    q_traj, ...
    t_vec, ...
    visualization_mode = 'position', ...
    camera_view = [-0.1 90])

%% Task-Space Motion
% Define Waypoints respectively Start and Goal End-Effector Positions
endeffector_points = [end_effector_position_start(1) end_effector_position_goal(1);
    end_effector_position_start(2) end_effector_position_goal(2);
    end_effector_position_start(3) end_effector_position_goal(3)]

% Duration of Motion in s
% Your code

% Start and End Time of Trajectory in s
% Your code

% Sample Time in s
% Your code

% Vector with Time Steps in s
% Your code

% Generate Cubic Task-Space Trajectory
% Your code

% Inverse Kinematics: Transfer Task-Space Trajectory to Joint-Space
% Trajectory
% Your code

% Visualize Manipulator Movement
figure()
visualize_manipulator(manipulator_rigidBodyTree, ...
    q_traj, ...
    t_vec, ...
    visualization_mode = 'position', ...
    camera_view = [-0.1 90])


%% Function Definitions
function T_base_endeffector = forward_kinematics(T_0_3, q)
    T_base_endeffector = double(T_0_3(q(1), q(2), q(3), q(4), q(5), q(6)));
end

function joint_variable_vector = inverse_kinematics(T_0_3, ...
    desired_position, ...
    config_requested)

    % Create Symbolic Variables
    syms q_1 q_2 q_3 q_4 q_5 q_6
    
    % Convert Symbolic Function For Extracting Single Equations
    T_0_3 = T_0_3(q_1, q_2, q_3, q_4, q_5, q_6);
    
    % Extract Equations For x-, y-, and z-Positions
    eqn_x = T_0_3(1, 4);
    eqn_y = T_0_3(2, 4);
    eqn_z = T_0_3(3, 4);
    
    % Equation For Desired x-, y-, and z-Positions
    eqn_desired_x = (eqn_x == desired_position(1));
    eqn_desired_y = (eqn_y == desired_position(2));
    eqn_desired_z = (eqn_z == desired_position(3));
    
    % Define Axes Limits
    deg = pi/180;
    assume((q_1 > -180*deg) & (q_1 <= 180*deg));
    assume((q_2 > -180*deg) & (q_2 <= 180*deg));
    assume((q_3 >= -0.1) & (q_3 <= 0.1));
    
    % Solve System of Equations
    S = solve(eqn_desired_x, ...
        eqn_desired_y, ...
        eqn_desired_z, ...
        ReturnConditions = true);
    
    % Check Number of Solutions
    % Your Code
    
    % Extract Solutions
    % Your Code
    
    % elseif (length(S.q_1) == 0)
    %     n_solutions = 0;
    %     error('Endeffector position can not be reached!')
    % else
    %     error('You have more than two solutions --> here seems to be something wrong!')
    % end
    
    % Chose Requested Configuration
    % Your Code

end