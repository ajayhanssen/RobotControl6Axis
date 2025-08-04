n = numel(homeConfiguration(robot));
lower = zeros(n,1);
upper = zeros(n,1);

for i = 1:n
    joint = robot.Bodies{i+1}.Joint;
    if strcmp(joint.Type, 'revolute') || strcmp(joint.Type, 'prismatic')
        limits = joint.PositionLimits;
        lower(i) = limits(1);
        upper(i) = limits(2);
    else
        lower(i) = -inf;
        upper(i) = inf;
    end
end

q = [1.7; -0.4; 1; -1; 1.7; 3];

isValid = all(q >= lower) && all(q <= upper)