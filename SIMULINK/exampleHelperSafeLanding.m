classdef exampleHelperSafeLanding < matlab.System
    %exampleHelperSafeLanding Compute safe landing zone using the map

    % Copyright 2023 The MathWorks, Inc.

    properties (Nontunable)
        innerBlockSize = [20 20];
        radius = 5;
        gridSize = 3;
        minPoints = 10;

        verticalVarianceThreshold = 0.5;
        slopeThreshold = 6;
        residualThreshold = 10;
        reliefThreshold = 3;

        weight = [1 1 1 1];
        qualityThreshold = 1; % Between 0 and 1

        pcDownsamplePercent = 0.1;

    end

    methods(Access = protected)
        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system with optional
            % outputs
            num = 3;
        end

        function [sz1, sz2, sz3] = getOutputSizeImpl(obj)
            % Return size for each output port
            sz1 = [1 1];
            sz2 = [1 3];
            sz3 = [1 1];
        end

        function [dt1, dt2, dt3] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            dt1 = "boolean";
            dt2 = "double";
            dt3 = "boolean";
        end

        function [o1, o2, o3] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            o1 = false;
            o2 = false;
            o3 = false;
        end

        function [o1, o2, o3] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            o1 = true;
            o2 = true;
            o3 = true;
        end

        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
          
        end

        function [isDetected, bestLandingZone, isSuitable] = stepImpl(obj, ptCloudMap, uavStartPosition)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.

            isDetected = false;
            ptCloudMap = pointCloud(ptCloudMap);
            ptCloudMap = removeInvalidPoints(ptCloudMap);
            ptCloud = pcdownsample(ptCloudMap, "random", obj.pcDownsamplePercent);

            
            [labels, scores] = obj.safeLandingZoneDetection(ptCloud, ...
                obj.innerBlockSize, obj.radius, obj.gridSize, ...
                obj.minPoints, obj.verticalVarianceThreshold, ...
                obj.slopeThreshold, obj.residualThreshold, ...
                obj.reliefThreshold);

            bestLandingZone = [0 0 0]; 
            isSuitable = false;

            if ~isempty(scores) || ~sum(all(isinf(scores))) == 4 || ~sum(all(isnan(scores))) == 4
                scaledWeight = obj.weight./sum(obj.weight);
                [bestLandingZone, isSuitable] = obj.bestLandingZoneWrapper( ...
                    uavStartPosition, ptCloud, labels, scores, scaledWeight, ...
                    obj.qualityThreshold);
                isDetected = true;

                

                figure;
                pcshow(ptCloudMap);
                hold on;
                pcshow(uavStartPosition',[1 0 0], MarkerSize=500);
                pcshow(bestLandingZone,[0 1 0], MarkerSize=500);
                
                hold off;
            end
        end

        function [labels, finalScores] = safeLandingZoneDetection(obj, ptCloud, innerBlockSize, radius, gridSize, minPoints, verticalVarianceThreshold, slopeThreshold, residualThreshold, reliefThreshold)

            finalScores = [];

            [outerBlockIndices,innerBlockIndices, ...
                innerBlockToOuterBlock,innerBlockIndFromLabels, ...
                innerBlockIndFromOuterBlock,boundaryIndices] = obj.helperOverlappingBlocks(ptCloud, ...
                radius,innerBlockSize);

            % Store the label of each point in a labels array
            labels = nan(ptCloud.Count,1);

            % Identify the ground points
            flatIdx = segmentGroundSMRF(ptCloud);

            % Mark the non-ground points as dangerous
            labels(~flatIdx) = 1;

            % Identify the unlabeled boundary points
            unlabeledBoundaryIndices = isnan(labels(boundaryIndices));

            % Label these points as unsuitable
            labels(boundaryIndices(unlabeledBoundaryIndices)) = 2;

            % Identify the indices of the points near the dangerous points
            unsuitableIdx = obj.helperUnsuitablePoints(ptCloud,labels,gridSize);

            % Label these points as unsuitable
            labels(unsuitableIdx) = 2;

            % Store the indices of the nearest neighboring points around each point
            % within the specified radius
            totalNeighborInd1 = {};

            % Perform block-by-block processing
            for curBlockIdx = 1:numel(outerBlockIndices)
                % Extract the inner block point cloud from the input point cloud
                innerBlockPtCloud = select(ptCloud,innerBlockIndices{curBlockIdx});

                % Extract the outer block point cloud from the input point cloud
                outerBlockPtCloud = select(ptCloud,outerBlockIndices{curBlockIdx});

                % Extract the labels of the outer block
                curOuterBlockLabels = labels(outerBlockIndices{curBlockIdx});

                % Create a mapping from the inner block labels to the outer block labels
                curInnerBlockToOuterBlock = innerBlockToOuterBlock{curBlockIdx};

                [labeledOuterBlock,neighborInd] = obj.helperNeighborIndices( ...
                    innerBlockPtCloud,outerBlockPtCloud,curOuterBlockLabels, ...
                    curInnerBlockToOuterBlock,minPoints,radius);

                % Store the neighbor indices of the points belonging to the inner block
                totalNeighborInd1 = [totalNeighborInd1; neighborInd];

                labels(innerBlockIndFromLabels{curBlockIdx}) = labeledOuterBlock( ...
                    innerBlockIndFromOuterBlock{curBlockIdx});
            end

            innerBlockIndicesCount = 1;
            for curBlockIdx = 1:numel(outerBlockIndices)
                % Extract the inner block point cloud from the input point cloud
                innerBlockPtCloud = select(ptCloud,innerBlockIndices{curBlockIdx});

                % Extract the outer block point cloud from the input point cloud
                outerBlockPtCloud = select(ptCloud,outerBlockIndices{curBlockIdx});

                % Extract the labels of the outer block
                curOuterBlockLabels = labels(outerBlockIndices{curBlockIdx});

                % Map the inner block labels to the outer block lables
                curInnerBlockToOuterBlock = innerBlockToOuterBlock{curBlockIdx};

                [labeledOuterBlock,updatedInnerBlockIndicesCount, ...
                    updatedTotalNeighborInd,curScores] = obj.helperLabelRiskyPts( ...
                    innerBlockIndicesCount,totalNeighborInd1,innerBlockPtCloud, ...
                    outerBlockPtCloud,curOuterBlockLabels, ...
                    curInnerBlockToOuterBlock, ...
                    verticalVarianceThreshold,slopeThreshold,residualThreshold,...
                    reliefThreshold);

                finalScores = [finalScores; curScores];
                totalNeighborInd1 = updatedTotalNeighborInd;
                innerBlockIndicesCount = updatedInnerBlockIndicesCount;

                labels(innerBlockIndFromLabels{curBlockIdx}) = labeledOuterBlock( ...
                    innerBlockIndFromOuterBlock{curBlockIdx});
            end

            % Label the remaining unlabeled points as suitable
            labels(isnan(labels)) = 4;
        end

        function unsuitableIdx = helperUnsuitablePoints(obj, ptCloud,labels,gridSize)
            %helperUnsuitablePoints Divides the point cloud into square grids of size
            % gridSize. If non-flat points are present in a grid, it computes the
            % indices of all the unlabeled flat points in that grid
            %
            %   This is an example helper function that is subject to change or removal
            %   in future releases.

            
            unsuitableIdx = false(numel(labels),1);

            numGridsX = round(diff(ptCloud.XLimits)/gridSize);
            numGridsY = round(diff(ptCloud.YLimits)/gridSize);
            gridLabels = false(numGridsX,numGridsY);
            indices = pcbin(ptCloud,[numGridsX numGridsY 1]);

            for r = 1:numGridsX
                for c = 1:numGridsY
                    idx = indices{r,c};
                    currentGridLabels = labels(idx);
                    % Check if the grid contains dangerous points and unlabeled points
                    if (any(currentGridLabels == 1) && any(isnan(currentGridLabels)))
                        gridLabels(r,c) = true;

                        % Find the indices of unlabeled points in the current grid
                        unsuitableIdx(idx(isnan(labels(idx)))) = true;
                    end
                end
            end

            % Perform morphological closing operation, to close the gaps in the point
            % cloud grids
            se = strel('square',gridSize);
            closedGridLabels = imclose(gridLabels,se);
            for r = 1:numGridsX
                for c = 1:numGridsY
                    if closedGridLabels(r,c) && ~(gridLabels(r,c))
                        idx = indices{r,c};
                        unsuitableIdx(idx(isnan(labels(idx)))) = true;
                    end
                end
            end
        end

        function [outerBlockIndices,innerBlockIndices, ...
                innerBlockToOuterBlock,innerBlockIndFromLabels, ...
                innerBlockIndFromOuterBlock,boundaryIndices] = helperOverlappingBlocks(obj, ptCloud, ...
                radius,innerBlockSize)
            %helperOverlappingBlocks Computes the indices required for overlapping
            % block processing
            %
            %   This is an example helper function that is subject to change or removal
            %   in future releases.

            

            % Define the ROI for the interior points
            interiorLimits = [ptCloud.XLimits(1)+radius, ptCloud.XLimits(2)-radius, ...
                ptCloud.YLimits(1)+radius, ptCloud.YLimits(2)-radius, ...
                ptCloud.ZLimits];

            % Extract the interior point cloud
            interiorIndices = findPointsInROI(ptCloud,interiorLimits);
            interiorPtCloud = select(ptCloud,interiorIndices);

            % Compute the indices of the boundary point cloud
            boundaryIndices = setdiff(1:ptCloud.Count,interiorIndices);

            % Divide the interior point cloud into blocks of size inner block size.
            % Compute the number of grids along the x and y axes
            numGridsIntX = round((diff(interiorPtCloud.XLimits))/(innerBlockSize(1)));
            numGridsIntY = round((diff(interiorPtCloud.YLimits))/(innerBlockSize(2)));

            % Compute the indices of each inner block
            [~,edgesX,edgesY,indx,indy] = histcounts2(interiorPtCloud.Location(:,1), ...
                interiorPtCloud.Location(:,2),[numGridsIntX,numGridsIntY], ...
                'XBinLimits',[ptCloud.XLimits(1)+radius,ptCloud.XLimits(2)-radius], ...
                'YBinLimits',[ptCloud.YLimits(1)+radius,ptCloud.YLimits(2)-radius]);
            innerBlockInd = sub2ind([numGridsIntX,numGridsIntY],indx,indy);

            % Compute the bin limits of the outer block along the x-axis
            startEdgesX = edgesX-radius;
            startEdgesX(1) = [];
            endEdgesX = edgesX+radius;
            endEdgesX(end) = [];
            xBinLimits = sort([ptCloud.XLimits(1),startEdgesX,endEdgesX,ptCloud.XLimits(2)]);

            % Compute the bin limits of the outer block along the y-axis
            startEdgesY = edgesY-radius;
            startEdgesY(1) = [];
            endEdgesY = edgesY+radius;
            endEdgesY(end) = [];
            yBinLimits = sort([ptCloud.YLimits(1),startEdgesY,endEdgesY,ptCloud.YLimits(2)]);

            % Compute the indices of each grid
            [n,~,~,indx,indy] = histcounts2(ptCloud.Location(:,1), ...
                ptCloud.Location(:,2),xBinLimits,yBinLimits);
            sz = size(n);
            outerBlockInd = sub2ind(sz,indx,indy);

            % Compute the start and end indices of grids constituting each outer grid
            startIdxX = 2*(1:numGridsIntX)-1;
            endIdxX = startIdxX+2;
            if numel(startIdxX) ~= numel(endIdxX)
                endIdxX(end) = [];
            end
            startIdxY = 2*(1:numGridsIntY)-1;
            endIdxY = startIdxY +2;
            if numel(startIdxY) ~= numel(endIdxY)
                endIdxY(end) = [];
            end

            % Traverse all the blocks
            colIdx = 1; rowIdx = 1; count = 1;
            innerBlockToOuterBlock = cell(numel(startIdxX)*numel(startIdxY),1);
            outerBlockIndices = cell(numel(startIdxX)*numel(startIdxY),1);
            innerBlockIndices = cell(numel(startIdxX)*numel(startIdxY),1);
            innerBlockIndFromLabels = cell(numel(startIdxX)*numel(startIdxY),1);
            innerBlockIndFromOuterBlock = cell(numel(startIdxX)*numel(startIdxY),1);
            while colIdx ~= numel(startIdxY)+1
                row1 = startIdxX(rowIdx):endIdxX(rowIdx);
                col1 = startIdxY(colIdx):endIdxY(colIdx);

                row = repmat(row1,1,numel(col1));
                col = repelem(col1,numel(row1));
                ind1 = sub2ind(sz,row,col);

                idx1 = find(sum(outerBlockInd == ind1,2));
                idx2 = interiorIndices(innerBlockInd == count);

                [~,ia,ib] = intersect(idx1,idx2);

                % Store the relation between inner block indices and outer block indices
                innerBlockToOuterBlock{count} = find(ismember(idx1, idx1(ia)));

                % Store the indices of the outer blocks
                outerBlockIndices{count} = idx1;

                % Store the indices of the inner blocks
                innerBlockIndices{count} = idx2(ib);

                % Store the indices of the inner block with respect to the labels
                innerBlockIndFromLabels{count} = idx1(ia);

                % Store the indices of the inner block with respect to the outer block
                innerBlockIndFromOuterBlock{count} = ia;
                count = count+1;
                rowIdx = rowIdx + 1;
                if rowIdx == numel(startIdxX)+1
                    rowIdx = 1;
                    colIdx = colIdx + 1;
                end
            end
        end

        function [curLabels,curNeighborInd] = helperNeighborIndices(obj, ...
                innerBlockPtCloud,outerBlockPtCloud,curLabels, ...
                curInnerBlockLabelsToOuterBlockLabels,minPoints,radius)
            curNeighborInd = cell(innerBlockPtCloud.Count,1);
            for i = 1:innerBlockPtCloud.Count
                % Compute nearest neighbor for only the unlabeled points
                if isnan(curLabels(curInnerBlockLabelsToOuterBlockLabels(i)))

                    % Find the nearest neighbors within radius for each point
                    [indices,~] = findNeighborsInRadius(outerBlockPtCloud, ...
                        innerBlockPtCloud.Location(i,:),radius);

                    % If the number of neighbors is less than the minimum neighbors,
                    % label the point as unsuitable
                    if numel(indices) < minPoints
                        curLabels(curInnerBlockLabelsToOuterBlockLabels(i)) = 2;
                        continue
                    end

                    % If the point is unlabeled and there are dangerous points present in
                    % the vicinity of the point, label the point as unsuitable
                    if any(curLabels(indices) == 1) && ...
                            isnan(curLabels(curInnerBlockLabelsToOuterBlockLabels(i)))
                        curLabels(curInnerBlockLabelsToOuterBlockLabels(i)) = 2;
                        continue
                    end

                    curNeighborInd{i} = indices;
                end
            end
        end

        function [curLabels,innerBlockIndicesCount, ...
                totalNeighborInd,curScores] = helperLabelRiskyPts(obj, innerBlockIndicesCount, ...
                totalNeighborInd,innerBlockPtCloud,outerBlockPtCloud,curLabels, ...
                curInnerBlockLabelsToOuterBlockLabels, ...
                verticalVarianceThreshold,slopeThreshold,residualThreshold, ...
                reliefThreshold)

            curScores = nan(innerBlockPtCloud.Count,4);
            for i = 1:innerBlockPtCloud.Count
                indices = totalNeighborInd{innerBlockIndicesCount};
                innerBlockIndicesCount = innerBlockIndicesCount + 1;

                if ~isempty(indices) && ...
                        isnan(curLabels(curInnerBlockLabelsToOuterBlockLabels(i)))

                    % If the point has neighbors, the point is unlabeled. and there
                    % are unsuitable points in the vicinity of the point, label the
                    % point as risky
                    if any(curLabels(indices) == 2)
                        curLabels(curInnerBlockLabelsToOuterBlockLabels(i)) = 3;
                        totalNeighborInd{innerBlockIndicesCount-1} = [];
                        curScores(i,:) = [-inf -inf -inf -inf];
                        continue;
                    end

                    % Evaluate the point for the vertical variance attribute
                    verticalVariance = var(outerBlockPtCloud.Location(indices,3));
                    curScores(i,1) = verticalVariance;

                    % Evaluate the point for the relief attribute
                    relief = max(outerBlockPtCloud.Location(indices,3)) ...
                        -min(outerBlockPtCloud.Location(indices,3));
                    curScores(i,2) = relief;

                    % Perform the plane fitting operation on the point and its
                    % neighbors
                    [model,~,outlierIndices] = pcfitplane(pointCloud( ...
                        outerBlockPtCloud.Location(indices,:)), ...
                        0.3,[0,0,1]);

                    % Evaluate the point for the slope attribute
                    slope = acosd(model.Normal(3));
                    curScores(i,3) = slope;

                    % Evaluate the point for the residual attribute
                    residual = rms(abs(outerBlockPtCloud.Location(outlierIndices,:) ...
                        *model.Parameters(1:3)' + model.Parameters(4)));
                    curScores(i,4) = residual;

                    if verticalVariance > verticalVarianceThreshold
                        curLabels(curInnerBlockLabelsToOuterBlockLabels(i)) = 3;
                        totalNeighborInd{innerBlockIndicesCount-1} = [];
                        continue;
                    end

                    if relief > reliefThreshold
                        curLabels(curInnerBlockLabelsToOuterBlockLabels(i)) = 3;
                        totalNeighborInd{innerBlockIndicesCount-1} = [];
                        continue
                    end

                    if slope > slopeThreshold
                        curLabels(curInnerBlockLabelsToOuterBlockLabels(i)) = 3;
                        totalNeighborInd{innerBlockIndicesCount-1} = [];
                        continue
                    end

                    if residual > residualThreshold
                        curLabels(curInnerBlockLabelsToOuterBlockLabels(i)) = 3;
                        totalNeighborInd{innerBlockIndicesCount-1} = [];
                        continue
                    end

                end
            end
            curScores(sum(isnan(curScores),2) == 4, :) = [];
        end

        function [bestLandingZone, isSuitable] = bestLandingZoneWrapper(obj, uavPosition, ptCloudMap, labels, scores, weight, qualityThreshold)
            % OUTPUT OVERVIEW
          
            isSuitable = false;
            bestLandingZone = [0 0 0];
            maxScores = max(scores,[],"omitnan");
            for i = 1:4
                idx = isnan(scores(:,i)) | isinf(scores(:,i));
                scores(idx,i) = maxScores(i);
            end
            scores = normalize(scores,"range");

            % Check if suitable points are absent
            if ~any(labels == 4)
                if ~any(labels == 3)
                    return;
                end

                % Set a bestLandingZone based on risky pts
                riskyZonePc = select(ptCloudMap, labels == 3);
                labels1 = labels(labels == 3 | labels == 4);
                labelsIdx = labels1 == 3;
                rangeSet = obj.helperComputeSafePt(riskyZonePc, scores, labelsIdx, weight, uavPosition);
                bestLandingZone = obj.helperFindBestZone(riskyZonePc,rangeSet,qualityThreshold);
                return;
            end

            % Set a bestLandingZone based on suitable pts
            isSuitable = true;
            safeZonePc = select(ptCloudMap, labels == 4);
            labels1 = labels(labels == 3 | labels == 4);
            labelsIdx = labels1 == 4;
            rangeSet = obj.helperComputeSafePt(safeZonePc, scores, labelsIdx, weight, uavPosition);
            bestLandingZone = obj.helperFindBestZone(safeZonePc,rangeSet,qualityThreshold);
        end

        function rangeSet = helperComputeSafePt(obj, pc, scores, labelsIdx, weight, uavLocation)
            scores = scores(labelsIdx, :);
            weightedScore = sum(weight.*scores,2);
            uavLocation = uavLocation(1:2);
            distanceFromUAV = sqrt(sum((pc.Location(:,1:2) - reshape(uavLocation,[],2)).^2,2));
            ptIdx = (1:pc.Count)';
            rangeSet = sortrows([weightedScore, distanceFromUAV, ptIdx]);
        end

        function bestLandingZone = helperFindBestZone(obj,pc,rangeSet,qualityThreshold)
            idx = rangeSet(:,1) <= qualityThreshold;
            if ~any(idx)
                bestLandingZone = pc.Location(rangeSet(1,end),:);
                return;
            end

            rangeSet = rangeSet(idx,:);
            rangeSet = sortrows([rangeSet(:,2) rangeSet(:,1) rangeSet(:,3)]);
            bestLandingZone = pc.Location(rangeSet(1,end),:);
        end
    end
end