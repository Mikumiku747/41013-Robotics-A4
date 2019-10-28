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
        %> Camera handle
        cam;
    end
    
    % Private properties
    properties (Access = private)
        
    end
    
    % Constants
    properties (Constant)
        %> Precision of Visual Servoing
        servoPrecision = 0.005;
        %> 'Center' coordinates
        centerCoords = [400 300];
        %> XY error scale
        xyErrorScale = diag([0.1 0.1]);
        %> Desired rectangle width
        rectWidth = 600;
        %> Z error scale
        zErrorScale = 0.1;
    end
    
    methods
        
        % Visual Servo so that a rectangle is in view
        function [done, error] = servoBetween(obj, robot, joints)
            % Update the camera to be at the robot's end effector
            obj.cam.T = robot.fkine(joints);
            % Project the 4 points onto the camera
            obj.cam.plot(obj.targetRectangle');
            projPoints = obj.cam.project(obj.targetRectangle')';
            % Calculate the center of those 4 points
            centerP = sum(projPoints)/size(projPoints,1);
            % Calculate the XY error
            error = nan(1,6);
            error(1:2) = (centerP - obj.centerCoords) * obj.xyErrorScale;
            % To calculate Z error ('closeness' error), we need to get the
            % size of the first edge of the rectangle
            width = norm(projPoints(1,:) - projPoints(2,:));
            error(3) = log(width/obj.rectWidth) * obj.zErrorScale;
            % Calculate the orientation and error to desired orientation
            % TODO above
            % Check if the positional error is within precision
            % TODO above
        end
        
    end
    
end
