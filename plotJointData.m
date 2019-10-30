%> @file plotJointData.m
%> @author Daniel Selmes
%> @date 2019-10-30

%> @brief Plots joint positions and velocities over time
function plotJointData(time, q, qVel, ~)
    % Position Data Figure
    figure(2);
    posPlot = tiledlayout(2,3);
    posPlot.Title.String = "Positon Data";
    posPlot.XLabel.String = "Time (s)";
    posPlot.YLabel.String = "Joint Position (rad)";
    for i = 1:6
        nexttile(i);
        plot(time(:), q(:,i), 'bo');
        ylim([-pi,pi]);
        title(sprintf("Joint %d Position", i));
    end
    figure(3);
    velPlot = tiledlayout(2,3);
    velPlot.Title.String = "Velocity Data";
    velPlot.XLabel.String = "Time (s)";
    velPlot.YLabel.String = "Joint Velocity (rad/s)";
    for i = 1:6
        nexttile(i);
        plot(time(:), qVel(:,i), 'ro');
        ylim([-pi/2,pi/2]);
        title(sprintf("Joint %d Velocity", i));
    end
    % Back to main figure
    figure(1);
end
