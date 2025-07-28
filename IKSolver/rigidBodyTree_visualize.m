function [] = rigidBodyTree_visualize( ...
    manipulator_rigidBodyTree, q_used, NameValueArgs)
%RIGIGBODYTREE_VISUALIZE rigidBodyTree_visualize(
% manipulator_rigidBodyTree, q_used) visualizes a serial-link manipulator
% represented by rigidBodyTree object for a given joint-variable vector
% q_used
%
%   Optional arguments are
%   - NameValueArgs.camera_view: Camera line of sight (default: [5 8])
%      - documentation: doc view
%
%   - NameValueArgs.axes_limits: Axes Limits 
%     (default: [-0.9 0.9 -0.9 0.9 0 0.6])
%      - documentation: doc axis
%   
%   - NameValueArgs.rigidBody_frame_to_turn_off: 
%     Numbers of rigidBody Frames to be Switched Off
%     (default: [])
%      - Frames Numbered From First to Last Link
%      - Turn Off Frame of rigidBodyTree Base
%
%   Author: benjamin.massow@mci.edu
%   Version: 26.11.2022

arguments
    %manipulator represented by a
    manipulator_rigidBodyTree;

    % Used Joint-Variable Vector
    q_used;

    % Camera Line of Sight
    NameValueArgs.camera_view ...
        (1,2) {mustBeNumeric(NameValueArgs.camera_view)} ...
        = [5 8];

    % Axes Limits
    NameValueArgs.axes_limits ...
        (1,6) {mustBeNumeric(NameValueArgs.axes_limits)} ...
        = [-0.9 0.9 -0.9 0.9 0 0.6];

    % Numbers of rigidBody Frames to be Switched Off
    % Numbered From First to Last Link
    NameValueArgs.rigidBody_frame_to_turn_off ...
        (1,:) {mustBeNumeric(NameValueArgs.rigidBody_frame_to_turn_off)} ...
        = [];
end

% Show rigidBodyTree Object at Joint Variable Values
show(manipulator_rigidBodyTree, q_used);

% Axis Labels
h_xlabel = xlabel({'$x$\,/\,m'});
set(h_xlabel, 'Interpreter', 'latex');

h_ylabel = ylabel({'$y$\,/\,m'});
set(h_ylabel, 'Interpreter', 'latex');

h_zlabel = zlabel({'$z$\,/\,m'});
set(h_zlabel, 'Interpreter', 'latex');

% Axes Limits and View
axis(NameValueArgs.axes_limits);
view(NameValueArgs.camera_view);

% Turn Off Patches of rigidBody Frames
if (~isempty(NameValueArgs.rigidBody_frame_to_turn_off))
    turn_off_rigidbody_frames(NameValueArgs.rigidBody_frame_to_turn_off)
end
end

function turn_off_rigidbody_frames(rigidBody_frame_to_turn_off)
% Patch Objects of rigidBody Frames
allPatches = findall(gcf, 'Type', 'Patch');

% Number of Frame Patches in Figure
n_frame_patches = length(allPatches);

% Turn Off Patches of rigidBody Frames
% Patches Are Numbered From Last Link to rigidBodyTree Base
for i = rigidBody_frame_to_turn_off
    allPatches(n_frame_patches - i).Visible = 'off';
end

% Turn Off Frame of rigidBodyTree Base
allPatches(n_frame_patches).Visible = 'off';
end