%> @file A4Task1.m
%> @author Daniel Selmes
%> @date 2019-20-23

%> brief Performs a demonstration of task 1 of assignment 4. 
function A4Task1()
    
    % Robot Base Position
    base = [1.254 5.253 1];
    % Drum offset from robot
    drumP = [0.25 -0.25 -1];
    % Visual Servo Target Positions
    targetP = [...
        0.205   0.650   0.600
        0.390   0.650   0.600
        0.205   0.530   0.600
        0.390   0.530   0.600
    ];
    % Translate the target positions by the drum position
    for i = 1:size(targetP,1)
        targetP(i,:) = targetP(i,:) + base + drumP;
    end
    
    % Startup
    figure(1);
    clf();
    hold on;
    
    % Create the robot and controller. 
    mdl_puma560;
    p560.base = transl(base(1), base(2), 1);
    Con = RobotController(p560, '/home/daniel/arte/robots/UNIMATE/puma560');
    % Initial Pose based on combination of teach and ikcon
    q0 = deg2rad([48.87 58.09 -139.4 -0.005 -98.65 48.86]);
    p560.animate(q0);
    Con.joints = q0;
    
    % Create the drum. 
    drumTrans = p560.base * transl(drumP(1), drumP(2), drumP(3));
    drum = PolyModel('Drum.ply', drumTrans);
    
    % Create the camera, attach to the end of the robot.
    % We want the end effector to remain 0.3 meters from the points, and
    % for the points to take up most of the image at this distance.
    % This means we should have a focal distance of 0.3, and since the size
    % of the rectangle we are tracking is approx. 0.4 m wide and our camera
    % horizontal resolution is 800, that means we want 0.4 m / 800 px or
    % 5e-4 m / px
    cam = CentralCamera('focal', 0.15, 'resolution', [800 600], ...
        'pixel', 5e-4, 'pose', p560.fkine(p560.getpos));
    %cam.plot_camera();
    
    % Create the visual servoing controller
    vs = VServ();
    vs.targetRectangle = targetP;
    vs.cam = cam;
    [k, r] = tr2angvec(p560.fkine(Con.joints));
    vs.targetAngles = r .* k;
    
    % Adjust the plot so the robot is centered
    xlim([base(1)-1, base(1)+1]);
    ylim([base(2)-1, base(2)+1]);
    zlim([0 2]);
    view([45 45]);
    
    % Start by using visual servoing to dynamically control the robot until
    % it is above the rectangle with points taking up most of the view
    dynamicControl(Con, p560, vs, @servoBetween);
    
    % Next, visual servo to each of the four points
    vs.targetPoint = targetP(1,:);
    dynamicControl(Con, p560, vs, @servoTo);
    vs.targetPoint = targetP(2,:);
    dynamicControl(Con, p560, vs, @servoTo);
    vs.targetPoint = targetP(3,:);
    dynamicControl(Con, p560, vs, @servoTo);
    vs.targetPoint = targetP(4,:);
    dynamicControl(Con, p560, vs, @servoTo);
    
end