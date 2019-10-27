%> @file PolyModel.m
%> @author Daniel Selmes
%> @date 2019-10-23

%> @brief Encapsulates model loading and rendering
classdef PolyModel < handle
    
    % Public properties
    properties (Access = public)
        %> Object face data
        faces;
        %> Object vertex data
        vertices;
        %> Object color data
        color;
        %> Object position & orientation
        trans;
        %> Patch object produced by plot
        plotP;
    end
    
    % Private properties
    properties (Access = private)
        
    end
    
    % Constants
    properties (Constant)
        
    end
    
    methods
        
        %> @breif Loads a .PLY model into the simulation
        %> 
        %> @param modelPath Path to the .ply file.
        %> @param trans Initial position/orientation of the model
        function obj = PolyModel(modelPath, trans)
            % Load in data
            [obj.faces, obj.vertices, data, ~] = plyread(modelPath);
            obj.color = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Load transform (also renders)
            obj.trans = trans;
        end
        
        %> @brief Plots the model
        function plotModel(obj)
            % Transform all the points
            count = size(obj.vertices, 1);
            newVertices = nan(count, 3);
            for i = 1:count
                t = eye(4);
                t(1:3,4) = obj.vertices(i,:);
                t = t * obj.trans;
                newVertices(i,:) = t(1:3,4);
            end
            % Render the object, store the plot handle
            if ~isempty(obj.plotP)
                delete(obj.plotP);
            end
            obj.plotP = trisurf(obj.faces, newVertices(:,1), ...
                newVertices(:,2), newVertices(:,3), ...
                'FaceVertexCData',obj.color, 'EdgeColor', 'interp', ...
                'EdgeLighting', 'flat');
        end
        
        %> @breif Re-plots the model when it is moved.
        %> @param newTrans The new model transform.
        function set.trans(obj, newTrans)
            obj.trans = newTrans;
            obj.plotModel();
        end
        
    end
end