function plotRigidBodyTreeFrames(robot, q, ax, scale)
% Plottet nur die Koordinatensysteme der 'link'-Bodies in einem rigidBodyTree
%
% Eingabe:
%   robot : rigidBodyTree-Objekt
%   q     : Gelenkwinkel (1xN-Vektor, optional; default: zeros)
%   ax    : axes-Handle für plot3 (z. B. UIAxes oder klassische axes)
%   scale : Skalierungsfaktor für die Achsenlänge (optional; default: 0.05)
    if nargin < 2 || isempty(q)
        q = zeros(1, numel(robot.homeConfiguration))';
    end
    if nargin < 3 || isempty(ax)
        figure;
        ax = axes;
    end
    if nargin < 4
        scale = 0.05;
    end

    hold(ax, 'on');

    link = 0;
    
    % Alle Bodies durchgehen
    for i = 1:robot.NumBodies
        body = robot.Bodies{i};
        name = body.Name;

        % Nur Bodies plotten, deren Name mit 'rigidBody_link_' beginnt
        if startsWith(name, 'rigidBody_link_')
            T = getTransform(robot, q, name);
            p = T(1:3, 4);
            R = T(1:3, 1:3);

            % Lokale Achsen
            x_axis = p + scale * R(:,1);
            y_axis = p + scale * R(:,2);
            z_axis = p + scale * R(:,3);

            % Plot
            plot3(ax, [p(1) x_axis(1)], [p(2) x_axis(2)], [p(3) x_axis(3)], 'r', 'LineWidth', 1.5);
            plot3(ax, [p(1) y_axis(1)], [p(2) y_axis(2)], [p(3) y_axis(3)], 'g', 'LineWidth', 1.5);
            plot3(ax, [p(1) z_axis(1)], [p(2) z_axis(2)], [p(3) z_axis(3)], 'b', 'LineWidth', 1.5);

            % Optional: Namen anzeigen
            name = sprintf("%d", link);
            text(ax, p(1), p(2), p(3), ['  ', name], 'FontSize', 8);
            link = link + 1;
        end
    end

    axis(ax, [-500e-3 500e-3 -500e-3 500e-3 0 500e-3]);
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    title(ax, 'Robot View');
    view(ax, 3);
    grid(ax, 'on');
end
