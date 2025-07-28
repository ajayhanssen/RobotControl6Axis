function manipulator = createKinematicChain(DH_parameter)
    DH_parameter = ...
        [DH_parameter(:,3) DH_parameter(:,4) DH_parameter(:,2) DH_parameter(:,1)];
    
    % create rigidbody object yallah
    manipulator_rigidBodyTree = rigidBodyTree(dataFormat = 'column');
    
    % Link 0 Fixed to Base
    rigidBody_link_0 = rigidBody('rigidBody_link_0');
    joint_base = rigidBodyJoint('joint_base', 'fixed');
    rigidBody_link_0.Joint = joint_base;

    % Add Link 0 on Base of RigidBodyTree Object
    addBody(manipulator_rigidBodyTree, rigidBody_link_0, manipulator_rigidBodyTree.BaseName);

    idx = 1
    for joint = 1:size(DH_parameter,1)/2
        
        rigidbodyJointOffset = rigidBody('rigidBody_joint_'+joint+'_offset');
        jointOffset = rigidBodyJoint('joint_'+joint+'_offset', 'fixed');
        setFixedTransform(jointOffset, DH_parameter(idx,:),'dh');
        rigidbodyJointOffset.Joint = jointOffset;
        idx = idx + 1;

        rigidbodyLink = rigidBody('rigidBody_link_'+joint);
        rigidbodyJoint = rigidBodyJoint('q_'+joint, 'revolute');
        setFixedTransform(rigidbodyJoint, DH_parameter(idx,:),'dh')
        rigidbodyLink.Joint = rigidbodyJoint;
        idx = idx + 1;

        addBody(manipulator_rigidBodyTree, rigidbodyJointOffset, rigidbodyLink)

    end
end

% Add Offset of Joint 1 on Link 0
addBody(manipulator_rigidBodyTree, rigidBody_joint_1_offset, rigidBody_link_0.Name);

% Add Link 1 on Offset of Joint 1
addBody(manipulator_rigidBodyTree, rigidBody_link_1, rigidBody_joint_1_offset.Name);