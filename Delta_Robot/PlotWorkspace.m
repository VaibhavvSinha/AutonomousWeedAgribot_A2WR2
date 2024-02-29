function PlotWorkspace(visitedPoints)
    figure;
    scatter3(visitedPoints(:, 1), visitedPoints(:, 2), visitedPoints(:, 3), 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Workspace of Delta Robot');
    grid on;
    axis equal;
end
