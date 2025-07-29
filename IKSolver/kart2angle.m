function angles = kart2angle(arduino, trans)
    load("robot_v2.mat");

    targetPos = [150e-3;
                 250e-3;
                 200e-3]
    
    targetPose = eye(4);
    targetPose(1:3,4) = targetPos;
    
    ik = inverseKinematics('RigidBodyTree', manipulator_rigidBodyTree);
    weights = [1 1 1 0.8 0.8 0.8]';
    initGuess = [0 -pi/2 pi 0 0 pi]';
    
    [configSol, solInfo] = ik('rigidBody_link_6', targetPose, weights, initGuess);
    configSol = configSol.*360./(2*pi)

    for i = 1:length(configSol)
        write(arduino, configSol(i), 'uint16');
    end

end