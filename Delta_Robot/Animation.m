function animation_data = Animation(angles, traj, param, animation_data)
    % ANIMATION Make Plot animation
    % !!! Precalculate trajectory with forward kinematics

    % simulation step size
    N = size(angles, 2);
    dt = 0.1;

    for i = 1:N 
        tic

        % Pass the trajectory point and visitedPoints to PlotPosition
        animation_data = PlotPosition(traj(:, i), angles(:, i), param, animation_data);

        toc
        pause(dt);
    end
end
