classdef exampleHelperMapBuilder < matlab.System
    %exampleHelperMapBuilder Builds map from lidar scans

    % Copyright 2023 The MathWorks, Inc.

    properties (Nontunable)
        mapGridSize = 0.2;
        downsamplePercent = 0.1;
        quadCopterROI = [-2 2 -2 2 -2 2];
        sensorPoseWRTUAV = rigidtform3d([90 90 0],[0 0 0]);
    end

    properties(Access = private)
        CurScan
        vSet
        absTform
        viewId
    end

    methods(Access = protected)
        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system with optional
            % outputs
            num = 2;
        end

        % function [sz1, sz2, sz3] = getOutputSizeImpl(obj)
        function [sz1, sz2] = getOutputSizeImpl(obj)
            % Return size for each output port
            sz1 = [1 1];
            sz2 = [600000 3];
        end

        % function [dt1, dt2, dt3] = getOutputDataTypeImpl(obj)
        function [dt1, dt2] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            dt1 = "boolean";
            dt2 = "double";
        end

        % function [o1, o2, o3] = isOutputComplexImpl(obj)
        function [o1, o2] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            o1 = false;
            o2 = false;
        end

        % function [o1, o2, o3] = isOutputFixedSizeImpl(obj)
        function [o1, o2] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            o1 = true;
            o2 = true;
        end

        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vSet = pcviewset;
            obj.absTform = rigidtform3d;
            obj.CurScan = pointCloud(zeros(32, 1083, 3));
            obj.viewId = 1;
        end

        function [isMapDone, pcMap] = stepImpl(obj, ptcloud, count)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.

            isMapDone = false;
            pcMap = nan(600000, 3);

            % Convert the location values to point cloud object
            curPc = pointCloud(ptcloud);

            % Transform the point cloud based on the lidar sensor pose
            % w.r.t. the UAV
            % curPc = pctransform(curPc,obj.sensorPoseWRTUAV);

            % Remove the quadcopter points from the point cloud
            quadCopterIdx = findPointsInROI(curPc, obj.quadCopterROI);
            allIndex = 1:curPc.Count;
            ia = ismember(allIndex, quadCopterIdx);
            curPc = select(curPc, ~ia', OutputSize="full");
            curDownsampledPc = pcdownsample(curPc, "random", obj.downsamplePercent);

            % Initialize the pcviewset object
            if obj.viewId == 1
                obj.vSet = addView(obj.vSet, obj.viewId, obj.absTform, "PointCloud", curPc);
                obj.CurScan = curDownsampledPc;
                obj.viewId = obj.viewId + 1;
                return;
            end

            % Take every 5th scan
            if rem(count,5) == 0

                prevDownsampledPc = obj.CurScan;

                relTform = pcregistericp(curDownsampledPc, prevDownsampledPc, Metric="planeToPlane");
                obj.absTform = rigidtform3d( obj.absTform.A * relTform.A );
                obj.vSet = addView(obj.vSet, obj.viewId, obj.absTform, "PointCloud", curPc);
                obj.vSet = addConnection(obj.vSet, obj.viewId-1, obj.viewId, relTform);

                obj.CurScan = curDownsampledPc;
                obj.viewId = obj.viewId + 1;
            end

            if rem(count, 50) == 0
                ptClouds = obj.vSet.Views.PointCloud;
                absPoses = obj.vSet.Views.AbsolutePose;
                pc = pcalign(ptClouds, absPoses, obj.mapGridSize);
                pc = pctransform(pc, rigidtform3d([90 90 180], [0 0 0]));
                pcshow(pc,Parent=gca);
            end

            if rem(count, 400) == 0
                ptClouds = obj.vSet.Views.PointCloud;
                absPoses = obj.vSet.Views.AbsolutePose;
                ptCloudMap = pcalign(ptClouds, absPoses, obj.mapGridSize);

                ptCloudMap = pctransform(ptCloudMap, rigidtform3d([90 90 180], [0 0 0]));
                
                isMapDone = true;
                pcMap(1:ptCloudMap.Count,:) = ptCloudMap.Location;

                obj.vSet = pcviewset;
                obj.vSet = pcviewset;
                obj.absTform = rigidtform3d;
                obj.CurScan = pointCloud(zeros(0, 0, 3));
                obj.viewId = 1;

            end
        end
    end
end
