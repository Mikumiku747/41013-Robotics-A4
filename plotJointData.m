%> @file plotJointData.m
%> @author Daniel Selmes
%> @date 2019-10-30

%> @brief Plots joint positions and velocities over time
function plotJointData(time, q, qVel, qAcc, tau)
    % Position Data Figure
    figure(2);
    posPlot = tiledlayout(2,3);
    posPlot.Title.String = "Positon Data";
    posPlot.XLabel.String = "Time (s)";
    posPlot.YLabel.String = "Joint Position (rad)";
    for i = 1:6
        nexttile(i);
        plot(time(:), q(:,i), 'b.-');
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
        plot(time(:), qVel(:,i), 'r.-');
        min_qV = min(qVel(:,i));
        max_qV = max(qVel(:,i));
        full = (max_qV-min_qV);
        ylim([min_qV-0.1*full, max_qV+0.1*full]);
        title(sprintf("Joint %d Velocity", i));
    end
    s = size(time,1);
    figure(4);
    accPlot = tiledlayout(2,3);
    accPlot.Title.String = 'Acceleration Data';
    accPlot.XLabel.String = 'Time (s)';
    accPlot.YLabel.String = 'Joint Acceleration (rad/s^2)';
    for i = 1:6
        nexttile(i);
        plot(time(1:s-1), qAcc(1:s-1,i), 'g.-');
        min_qA = min(qAcc(1:s-1,i));
        max_qA = max(qAcc(1:s-1,i));
        full = (max_qA - min_qA);
        ylim([min_qA - 0.1*full, max_qA + 0.1*full]);
        title(sprintf('Joint %d Acceleration', i));
    end
    figure(5);
    tauPlot = tiledlayout(2,3);
    tauPlot.Title.String = "Joint Torque (Nm)";
    tauPlot.XLabel.String = "Time (s)";
    tauPlot.YLabel.String = "Torque (Nm)";
    for i = 1:6
        nexttile(i);
        plot(time(1:s-1), tau(1:s-1,i), 'm.-');
        min_t = min(tau(1:s-1,i));
        max_t = max(tau(1:s-1,i));
        full = (max_t - min_t);
        ylim([min_t -  0.1*full, max_t + 0.1*full]);
        title(sprintf('Joint %d Torque', i));
    end
    % Back to main figure
    figure(1);
end
