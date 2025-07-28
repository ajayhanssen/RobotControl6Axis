function manipulator_rigidBodyTree = create_manipulator(DH_parameter)
%% Change Order of DH-Paramters
% Order Needed for rigidBodyTree object: [a alpha d phi]
DH_parameter = ...
    [DH_parameter(:,3) DH_parameter(:,4) DH_parameter(:,2) DH_parameter(:,1)];
                
%% Create rigidBody Objects and RigidBodyJoint Objects
% Link 0 Fixed to Base
rigidBody_link_0 = rigidBody('rigidBody_link_0');
joint_base = rigidBodyJoint('joint_base', 'fixed');
rigidBody_link_0.Joint = joint_base;

% Fixed Transform for Offset of Joint 1
rigidBody_joint_1_offset = rigidBody('rigidBody_joint_1_offset');
joint_1_offset = rigidBodyJoint('joint_1_offset', 'fixed');
setFixedTransform(joint_1_offset, DH_parameter(1,:),'dh');
rigidBody_joint_1_offset.Joint = joint_1_offset;

% Link 1 Moved by Revolute Joint 1
rigidBody_link_1 = rigidBody('rigidBody_link_1');
joint_1 = rigidBodyJoint('q_1', 'revolute');
setFixedTransform(joint_1, DH_parameter(2,:),'dh');
rigidBody_link_1.Joint = joint_1;

% Fixed Transform for Offset of Joint 2
rigidBody_joint_2_offset = rigidBody('rigidBody_joint_2_offset');
joint_2_offset = rigidBodyJoint('joint_2_offset', 'fixed');
setFixedTransform(joint_2_offset, DH_parameter(3,:),'dh');
rigidBody_joint_2_offset.Joint = joint_2_offset;

% Link 2 Moved by Revolute Joint 2
rigidBody_link_2 = rigidBody('rigidBody_link_2');
joint_2 = rigidBodyJoint('q_2', 'revolute');
setFixedTransform(joint_2, DH_parameter(4,:),'dh');
rigidBody_link_2.Joint = joint_2;

% Fixed Transform for Offset of Joint 3
rigidBody_joint_3_offset = rigidBody('rigidBody_joint_3_offset');
joint_3_offset = rigidBodyJoint('joint_3_offset', 'fixed');
setFixedTransform(joint_3_offset, DH_parameter(5,:),'dh');
rigidBody_joint_3_offset.Joint = joint_3_offset;

% Link 3 Moved by revolute Joint 3
rigidBody_link_3 = rigidBody('rigidBody_link_3');
joint_3 = rigidBodyJoint('q_3', 'revolute');
setFixedTransform(joint_3, DH_parameter(6,:),'dh');
rigidBody_link_3.Joint = joint_3;

% Fixed Transform for Offset of Joint 4
rigidBody_joint_4_offset = rigidBody('rigidBody_joint_4_offset');
joint_4_offset = rigidBodyJoint('joint_4_offset', 'fixed');
setFixedTransform(joint_4_offset, DH_parameter(7,:),'dh');
rigidBody_joint_4_offset.Joint = joint_4_offset;

% Link 4 Moved by Revolute Joint 4
rigidBody_link_4 = rigidBody('rigidBody_link_4');
joint_4 = rigidBodyJoint('q_4', 'revolute');
setFixedTransform(joint_4, DH_parameter(8,:),'dh');
rigidBody_link_4.Joint = joint_4;

% Fixed Transform for Offset of Joint 5
rigidBody_joint_5_offset = rigidBody('rigidBody_joint_5_offset');
joint_5_offset = rigidBodyJoint('joint_5_offset', 'fixed');
setFixedTransform(joint_5_offset, DH_parameter(9,:),'dh');
rigidBody_joint_5_offset.Joint = joint_5_offset;

% Link 5 Moved by Revolute Joint 5
rigidBody_link_5 = rigidBody('rigidBody_link_5');
joint_5 = rigidBodyJoint('q_5', 'revolute');
setFixedTransform(joint_5, DH_parameter(10,:),'dh');
rigidBody_link_5.Joint = joint_5;

% Fixed Transform for Offset of Joint 6
rigidBody_joint_6_offset = rigidBody('rigidBody_joint_6_offset');
joint_6_offset = rigidBodyJoint('joint_6_offset', 'fixed');
setFixedTransform(joint_6_offset, DH_parameter(11,:),'dh');
rigidBody_joint_6_offset.Joint = joint_6_offset;

% Link 6 Moved by Revolute Joint 6
rigidBody_link_6 = rigidBody('rigidBody_link_6');
joint_6 = rigidBodyJoint('q_6', 'revolute');
setFixedTransform(joint_6, DH_parameter(12,:),'dh');
rigidBody_link_6.Joint = joint_6;

%% Create rigidBodyTree Object
manipulator_rigidBodyTree = rigidBodyTree(dataFormat = 'column');

%% Add rigidBodies and Corresponding Joints to rigidBodyTree Object
% Add Link 0 on Base of RigidBodyTree Object
addBody(manipulator_rigidBodyTree, rigidBody_link_0, manipulator_rigidBodyTree.BaseName);

% Add Offset of Joint 1 on Link 0
addBody(manipulator_rigidBodyTree, rigidBody_joint_1_offset, rigidBody_link_0.Name);

% Add Link 1 on Offset of Joint 1
addBody(manipulator_rigidBodyTree, rigidBody_link_1, rigidBody_joint_1_offset.Name);


% Add Offset of Joint 2 on Link 1
addBody(manipulator_rigidBodyTree, rigidBody_joint_2_offset, rigidBody_link_1.Name);

% Add Link 2 on Offset of Link 1
addBody(manipulator_rigidBodyTree, rigidBody_link_2, rigidBody_joint_2_offset.Name);



% Add Offset of Joint 3 on Link 2
addBody(manipulator_rigidBodyTree, rigidBody_joint_3_offset, rigidBody_link_2.Name);

% Add Link 3 on Offset of Joint 3
addBody(manipulator_rigidBodyTree, rigidBody_link_3, rigidBody_joint_3_offset.Name);


% Add Offset of Joint 4 on Link 3
addBody(manipulator_rigidBodyTree, rigidBody_joint_4_offset, rigidBody_link_3.Name);

% Add Link 4 on Offset of Joint 4
addBody(manipulator_rigidBodyTree, rigidBody_link_4, rigidBody_joint_4_offset.Name);


% Add Offset of Joint 5 on Link 4
addBody(manipulator_rigidBodyTree, rigidBody_joint_5_offset, rigidBody_link_4.Name);

% Add Link 5 on Offset of Joint 5
addBody(manipulator_rigidBodyTree, rigidBody_link_5, rigidBody_joint_5_offset.Name);


% Add Offset of Joint 6 on Link 5
addBody(manipulator_rigidBodyTree, rigidBody_joint_6_offset, rigidBody_link_5.Name);

% Add Link 6 on Offset of Joint 6
addBody(manipulator_rigidBodyTree, rigidBody_link_6, rigidBody_joint_6_offset.Name);
end