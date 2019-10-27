%> @file A4Task1.m
%> @author Daniel Selmes
%> @date 2019-20-23

%> brief Performs a demonstration of task 1 of assignment 4. 
function A4Task1()
    
    % Robot Base Position
    base = [1.254, 5.253];
    % Drum offset from robot
    drumP = [0.25 -0.25 -1];
    % Visual Servo Target Positions
    targetP = [...
        0.205   0.650   0.600
        0.390   0.650   0.600
        0.205   0.530   0.600
        0.390   0.650   0.600
    ];
    
    % Startup
    figure(1);
    clf();
    hold on;
    
    % Create the robot and controller. 
    mdl_puma560;
    p560.base = transl(base(1), base(2), 1);
    Con = RobotController(p560, '/home/daniel/arte/robots/UNIMATE/puma560');
    q0 = deg2rad([25 90 -180 0 -90 25]);
    p560.animate(q0);
    
    % Create the drum. 
    drumTrans = p560.base * transl(drumP(1), drumP(2), drumP(3));
    drum = PolyModel('Drum.ply', drumTrans);
    
    % Adjust the plot so the robot is centered
    xlim([base(1)-1, base(1)+1]);
    ylim([base(2)-1, base(2)+1]);
    zlim([0 2]);
    view([45 45]);
    
    % Create the camera, attach to the end of the robot.
    cam = CentralCamera('focal', 0.15, 'resolution', [400 300], ...
        'pixel', 10e-5, 'pose', p560.fkine(p560.getpos));
    cam.plot_camera();
    cam.plot(targetP');
end