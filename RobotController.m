%> @file RobotController.m
%> @author Daniel Selmes
%> @date 2019-10-23

%> @brief Provides planning and control functions for a robot.
classdef RobotController < handle
    
    % Public properties
    properties (Access = public)
        %> Enables pretty 3d plotting
        use3DModel = true;
        %> Whether to actually plot the robot or not
        plotRobot = true;
        %> Internal representation of the joint robot state
        joints = [0 0 0 0 0 0];
        %> Logging function callback
        logCallback;
        %> Robot control frequency.
        controlFrequency = 20;
        %> Positional error correction speed
        dynamicPSpeed = 0.5; % 500 mm/s
        %> Rotational error correction speed
        dynamicRSpeed = pi/2; % pi/2 rad/s
        %> Minimim XYZ cartesian speed
        minXYZSpeed = 0.5 / 20;
    end
    
    % Private properties
    properties (Access = private)
        %> Path to the robot's models
        modelPath;
    end
    
    methods
        
        %> @brief Initialises the robot controller
        %> 
        %> All arguments are optional except for robot.
        %> @param robot The robot to use.
        %> @param modelPath Path to the robot's display models.
        function obj = RobotController(robot, modelPath, plotRobot)
            % Check if the path was provided
            if nargin < 2
                obj.modelPath = '';
                obj.use3DModel = false;
            else
                obj.modelPath = modelPath;
                obj.use3DModel = true;
            end
            if nargin < 3
                plotRobot = true;
            end
            obj.plotRobot = plotRobot;
            
            % Setup the controller
            if plotRobot
                if obj.use3DModel
                    robot.plot3d(zeros(1,size(robot.links,2)), ...
                        'path', obj.modelPath);
                else
                    robot.plot(zeros(1,size(robot.links,2)));
                end
            end
        end
        
        
        %> @brief Plans a joint movement to the given transform
        %> 
        %> @param robot The robot to plan the trajectory for.
        %> @param destT The destination transform.
        %> @param time Time in seconds for the trajectory to last.
        %> @return Trajectory from the current pose to destT.
        function trajectory = planJ(obj, robot, destT, time)
            
            % Compute the destination joint position.
            startQ = obj.joints();
            endQ = robot.ikcon(destT, startQ);
            
            % Create an interpolation between start and end
            diff = endQ - startQ;
            interp = lspb(0, 1, time*obj.controlFrequency);
            trajectory = nan(size(interp,1),size(robot.links, 2));
            for i = 1:size(interp,1)
                trajectory(i,:) = startQ + diff*interp(i);
            end
        end
        
        %> @brief Plots a point trajectory for the robot to follow
        %> 
        %> The frameCallback function, if supplied, is called once per
        %> movement frame to allow for other parts of the simulation to be
        %> run in sync with the robot during a movement. If this callback
        %> returns false, movement will end.
        %> @param robot The robot to animate
        %> @param traj The trajectory to follow
        %> @param frameCallback Called once per movement frame
        function plotJ(obj, robot, traj, frameCallback)
            
            % Prep for the callback
            if nargin > 3
                runCallBack = true;
            else
                runCallBack = false;
            end
            % Iteratively plot each of the joints
            for i = 1:size(traj,1)
                % Move the robot.
                if obj.plotRobot
                    robot.animate(traj(i,:));
                end
                obj.joints =  traj(i,:);
                % Run the callback if needed
                if runCallBack
                    if frameCallback()
                        return;
                    end
                end
            end
        end
        
        %> @brief Uses an error function to do linear dynamic control
        %> 
        %> The error function is passed the robot and the current joint
        %> positions as input arguments.
        %> The error function should return 2 values, a boolean describing
        %> whether the operation is complete and an X Y Z R P Y error
        %> function which specifies how the 
        %> 
        %> @param robot The robot to perform the move with
        %> @param errorController The class which generates the error
        %> values
        %> @param errorFunction The error function whihc guides the robot
        function dynamicControl(obj, robot, errorController, errorFunction)
            logTime = 0;
            while true
                % Run the error function
                [done, error] = errorFunction(errorController, robot, obj.joints);
                % Exit if OK!
                if done
                    return
                end
                % If the error is too small, kick it up a minimum (the
                % control function should set it zero for no movement).
                if norm(error(1:3)) < obj.minXYZSpeed
                    error(1:3) = (error(1:3) / norm(error(1:3))) * obj.minXYZSpeed;
                end
                % Calculate end effector velocity based on the error
                tVel = [error(1:3) * obj.dynamicPSpeed, ...
                    error(4:6) * obj.dynamicRSpeed];
                % Calculate joint velocity
                j0 = robot.jacob0(obj.joints);
                qVel = (inv(j0) * tVel')';
                % Apply the joint velocity
                obj.joints = obj.joints + qVel / obj.controlFrequency;
                robot.animate(obj.joints);
                % Do logging if required
                if ~isempty(obj.logCallback)
                    logTime = logTime + 1/obj.controlFrequency;
                    obj.logCallback(logTime, obj.joints, qVel, error);
                end
            end
        end
    end
    
end