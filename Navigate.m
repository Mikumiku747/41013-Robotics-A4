%> @file Navigate.m
%> @author Daniel Selmes
%> @date 2019-10-30

%> @brief Simple error function for cartesian movements
classdef Navigate < handle
    
    % Public Properties
    properties (Access = public)
        %> Destination position
        destPos;
        %> Destination angles
        destAng;
        % Controls how fast the movement occurs.
        errorProportion = 0.1;
    end
    
    % Constants
    properties (Constant)
        %> Positional Precision
        pPrecision = 0.0025;
        %> Rotational Precision
        rPrecision = pi/72;
    end
    
    methods
        
        %> @brief Constructor, sets the destination
        function obj = Navigate(destT)
            % Set the position
            obj.destPos = destT(1:3,4)';
            % Set the angles
            [k, angs] = tr2angvec(destT);
            obj.destAng = angs .* k;
        end
        
        %> @brief Navigate towards the desired point
        function [done, error] = navigateTo(obj, robot, joints)
            error = nan(1,6);
            % Calculate current position and angles
            currentT = robot.fkine(joints);
            pError = obj.destPos - currentT(1:3,4)';
            [k, angs] = tr2angvec(currentT);
            rError = (mod((angs .* k)+pi/2, 2*pi)-pi/2) ...
                - (mod(obj.destAng+pi/2, 2*pi)-pi/2);
            % Zero them if small enough
            if norm(pError) < obj.pPrecision
                error(1:3) = zeros(1,3);
            else
                error(1:3) = (pError / norm(pError))*obj.errorProportion;
            end
            for i = 1:3
                if rError(i) < obj.rPrecision
                    error(3+i) = 0;
                else
                    error(3+i) = rError(i);
                end
            end
            % Check if we are in position yet
            if norm(pError) < obj.pPrecision && rError(1) < obj.rPrecision ...
                && rError(2) < obj.rPrecision && rError(3) < obj.rPrecision
                done = true;
            else
                done = false;
            end
        end
        
    end
    
end