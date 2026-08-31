% ---------------- Native C inverse Kinematics testing in MATLAB environment ----------------
% ---------------- import rigidBodyTree ----------------
robot = importrobot('abbIrb120.urdf', 'DataFormat', 'row');

% ---------------- Initialization ----------------
NumberOfSteps = 250;

x = linspace(0.3186, 0.3, NumberOfSteps);
y = linspace(0, 0.3, NumberOfSteps);
z = linspace(0.1225, 0.3, NumberOfSteps);

lastValidConfig = [0, pi/4, pi/5, 0, 0, 0];
ikSolution = zeros(NumberOfSteps, 6);

% ---------------- Call mexFuntion ----------------

for k = 1 : NumberOfSteps

    targetPose = [x(k), y(k), z(k), 0, 0, 0];
    [jointAngles, T, info] = CallMexFunction(lastValidConfig, targetPose);

    if info.converged
        lastValidConfig = jointAngles.';
        ikSolution(k, :) = lastValidConfig;
    else
        ikSolution(k,:) = NaN(1,6);
    end

    fprintf(['Point %d/%d  converged=%d  iterations=%d  ', ...
         'posErr=%.3e  oriErr=%.3e\n'], ...
       k, NumberOfSteps, ...
    info.converged, ...
    info.iterations, ...
    info.positionError, ...
    info.orientationError);

    %show(robot, ikSolution(k,:), ...
        %'PreservePlot', false, ...
        %'Frames', 'on');

    %axis equal;
    %grid on;
    %view(135, 20);
    %drawnow; 
end

writematrix(rad2deg(ikSolution), ...
    'history_log.txt', 'Delimiter', 'tab');