%> @file VServ.m
%> @author Daniel Selmes
%> @date 2019-10-27

%> @breif Provides visual servoing capabilities
%> 
%> Visual servoing services are provided in the form of error functions,
%> which calculate error from a desired position so that the robot's
%> dynamic controller can move the robot to the right position.
classdef VServ < handle
    
    % Public properties
    properties (Access = public)
        %> Set of four points to servo into view
        targetRectangle;
        %> A single point to servo over
        targetPoint;
        %> Orientation to maintain whilst servoing (3x3 trot angles)
        targetAngles;
        %> Camera handle
        cam;
    end
    
    % Private properties
    properties (Access = private)
        
    end
    
    % Constants
    properties (Constant)
        %> Precision of Visual Servoing (in px)
        servoPrecision = 5;
        %> 'Center' coordinates
        centerCoords = [400 300];
        %> Desired rectangle width
        rectWidth = 300;
    end
    
    methods
        
        %> Visual Servo so that a rectangle is in view
        function [done, error] = servoBetween(obj, robot, joints)
            % Update the camera to be at the robot's end effector
            obj.cam.T = robot.fkine(joints);
            % Project the 4 points onto the camera
            obj.cam.plot(obj.targetRectangle');
            obj.cam.plot_camera('scale', 0.05);
            projPoints = obj.cam.project(obj.targetRectangle')';
            % Calculate the center of those 4 points
            centerP = sum(projPoints)/size(projPoints,1);
            % Calculate the XY error
            error = nan(1,6);
            xyError = (centerP - obj.centerCoords);
            error(1:2) = xyError * obj.cam.rho(1);
            % Correct the sense of the error vectors to match the tool
            % coordinate space
            error(1:2) = error(1:2) * [-1 0; 0 1];
%             fprintf('XY Pixel Error: %.2f\n', norm(xyError));
            % To calculate Z error ('closeness' error), we need to get the
            % size of the first edge of the rectangle
            width = norm(projPoints(1,:) - projPoints(2,:));
            widthRatio = log(width/obj.rectWidth);
            if abs(widthRatio) > 0.015
                error(3) = widthRatio;
            else
                error(3) = 0;
            end
%             fprintf('Z Ratio Error: %.2f\n', widthRatio);
            % Calculate the orientation and error to desired orientation
            [k, r] = tr2angvec(robot.fkine(joints));
            rAngles = r .* k;
            rError = rAngles - obj.targetAngles;
            for i = 1:size(rError, 2)
                if rError(i) < pi/72
                    error(3+i) = 0;
                else
                    error(3+i) = rError(i);
                end
            end
            % Check if the positional error is within precision
            if norm(xyError) > obj.servoPrecision || ...
                abs(widthRatio) > 0.015
                done = false;
            else
                done = true;
            end
        end
        
        %> Visual Servo across to a point
        function [done, error] = servoTo(obj, robot, joints)
            % Update the camera to be at the robot's end effector
            obj.cam.T = robot.fkine(joints);
            % Project the target onto the camera
            % Project the 4 points onto the camera
            obj.cam.plot(obj.targetPoint');
            obj.cam.plot_camera('scale', 0.05);
            projPoint = obj.cam.project(obj.targetPoint')';
            % Calculate the XY error
            error = nan(1,6);
            xyError = (projPoint - obj.centerCoords);
            error(1:2) = xyError * obj.cam.rho(1);
            % Correct the sense of the error vectors to match the tool
            % coordinate space
            error(1:2) = error(1:2) * [-1 0; 0 1];
            % We don't move Z during this type of servo
            error(3) = 0;
            % Calculate the orientation and error to desired orientation
            [k, r] = tr2angvec(robot.fkine(joints));
            rAngles = r .* k;
            rError = rAngles - obj.targetAngles;
            for i = 1:size(rError, 2)
                if rError(i) < pi/72
                    error(3+i) = 0;
                else
                    error(3+i) = rError(i);
                end
            end
            % Check if the positional error is within precision
            if norm(xyError) > obj.servoPrecision
                done = false;
            else
                done = true;
            end
        end
    end
    
end
