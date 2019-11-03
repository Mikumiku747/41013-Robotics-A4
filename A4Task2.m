%> @file A4Task2
%> @author Daniel Selmes
%> @date 2019-10-30

%> @brief performs a demonstration of task 2 of assignment 4
function [maxT2, maxT3] = A4Task2(eP)
    
    % Globals for data recording
    global data_time;
    global data_q;
    global data_qVel;
    global data_qAcc;
    global data_tau;
    global data_error;

    % Robot base position
    base = [1.254 5.253 1];
    % Drum offset from robot
    drumP = [0.25 -0.25 -1];
    % Movement targets (Based on part 1, the first two single waypoints)
    initialQ = [0 pi/2 -pi/2 0 0 0];
%     startT = ...
%        [-1.0000   -0.0002    0.0005    1.7091
%         -0.0002    1.0000    0.0005    5.6517
%         -0.0005    0.0005   -1.0000    0.7864
%          0         0         0         1.0000];
    startT = transl(1.7091, 5.6517, 0.7864) ...
        * angvec2tr(pi, [0.0000 -1.0000 0.0000]);
%     endT = ...
%        [-1.0000   -0.0002    0.0005    1.8917
%         -0.0002    1.0000    0.0005    5.6540
%         -0.0005    0.0005   -1.0000    0.7869
%               0         0         0    1.0000];
    endT = transl(1.8917, 5.6540, 0.7869) ...
        * angvec2tr(pi, [0.0000 -1.0000 0.0000]);
    
    % Startup
    figure(1);
    clf
    hold on
    
    % Clear out the logging variables
    data_time = [];
    data_q = [];
    data_qVel = [];
    data_qAcc = [];
    data_tau = [];
    data_error = [];
    
    % Create the robot and controller. 
    mdl_puma560;
    p560.base = transl(base(1), base(2), 1);
    Con = RobotController(p560, '/home/daniel/arte/robots/UNIMATE/puma560');
    Con.logCallback = @saveJointData;
    Con.controlFrequency = 200*eP;
    % Initial Pose is just straight in the air
    p560.animate(initialQ);
    Con.joints = initialQ;
    
    % Create the drum. 
    drumTrans = p560.base * transl(drumP(1), drumP(2), drumP(3));
    drum = PolyModel('Drum.ply', drumTrans);
    
    % Adjust the plot so the robot is centered
    xlim([base(1)-1, base(1)+1]);
    ylim([base(2)-1, base(2)+1]);
    zlim([0 2]);
    view([45 45]);
    
    % Joint move into the starting position
    startQ = p560.ikcon(startT, initialQ);
    Con.joints = startQ;
    p560.animate(startQ);
    
    % Use the dynamicControl function to move to the endpoint
    nv = Navigate(endT);
    nv.errorProportion = eP;
    dynamicControl(Con, p560, nv, @navigateTo);
    
    % Calculate acceleration data
    s = size(data_time,1);
    dt = 1/(data_time(2) - data_time(1));
    data_qAcc = (data_qVel(2:s,:) - data_qVel(1:s-1,:)) .* dt;
    
    % Calculate torque data
    for i = 1:s-1
        p560.payload(2.2, [0, 0, 0.1]);
        tau_m = abs(p560.inertia(data_q(i,:)) * data_qAcc(i,:)');
        tau_c = abs(p560.coriolis(data_q(i,:), data_qVel(i,:)) * data_qVel(i,:)');
        tau_g = abs(p560.gravload(data_q(i,:))');
        data_tau = [data_tau; tau_m' + tau_c' + tau_g'];
    end
    
    % Plot joint data
    plotJointData(data_time, data_q, data_qVel, data_qAcc, data_tau);
    
    maxT2 = max(data_tau(:,2));
    maxT3 = max(data_tau(:,3));

end