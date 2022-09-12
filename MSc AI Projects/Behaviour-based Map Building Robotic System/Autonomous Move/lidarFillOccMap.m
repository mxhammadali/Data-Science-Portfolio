% INPUT: Current occupancy map and other CoppeliaSim
% parameters
% OUTPUT: Updated occupancy map from Laser Range Finder Data
function currOccMap = lidarFillOccMap(currOccMap, csim, clientID, sensor, maxXPos, maxYPos, xAxisCorrectionFactor, yAxisCorrectionFactor)
    arguments
        currOccMap % binaryOccupancyMap obj
        csim % CoppeliaSim remote api obj
        clientID % client id of the established MATLAB -> CSIM connection
        sensor CsimLidarSensor
        maxXPos double
        maxYPos double
        xAxisCorrectionFactor double
        yAxisCorrectionFactor double
    end
    
    % FUNCTION BEGIN

    if ~sensor.Initialised
        % CoppeliaSim recommends first call to be simx_opmode_streaming and
        % subsequent calls as simx_opmode_buffer for simxGetStringSignal remote
        % api
        [~,~] = csim.simxGetStringSignal(clientID,sensor.PointCloudDataPointer,csim.simx_opmode_streaming);
        [~,~] = csim.simxGetStringSignal(clientID,sensor.LaserOriginDataPointer,csim.simx_opmode_streaming);
        [~,~] = csim.simxGetStringSignal(clientID,sensor.LaserOrientationDataPointer,csim.simx_opmode_streaming);
        
        sensor.Initialised = 1;
    end

    % Read point cloud data signal
    [respCodePointCloud,pointCloud] = csim.simxGetStringSignal( ...
            clientID,sensor.PointCloudDataPointer,csim.simx_opmode_buffer);
    
    % Read sensor origin and orientation signal
    % Both sensor origin and orientation is required at the instant
    % when laser point cloud is collected. Therefore, DO NOT use
    % remote api functions to obtain these values
    [respCodeOrigin,laserSensorOrigin] = csim.simxGetStringSignal( ...
        clientID,sensor.LaserOriginDataPointer,csim.simx_opmode_buffer);
    [respCodeOrient,laserSensorOrientation] = csim.simxGetStringSignal( ...
        clientID,sensor.LaserOrientationDataPointer,csim.simx_opmode_buffer);
    
    if(csim.simx_return_ok == respCodePointCloud && ...
            csim.simx_return_ok == respCodeOrigin && ...
            csim.simx_return_ok == respCodeOrient)

        % 3 (x,y,z) x 684 datapoints (this value is mentioned in vrep child script)
        % output is a 1 x 2052 vector (3 x 684 = 2052)
        pointCloudVec = double(csim.simxUnpackFloats(pointCloud));

        % origin consists of the vector [x, y, z]
        hokuyoOrigin = double(csim.simxUnpackFloats(laserSensorOrigin));
        
        % orientation consists of vector of 3 euler angles
        % https://www.coppeliarobotics.com/helpFiles/en/eulerAngles.htm
        hokuyoOrient = double(csim.simxUnpackFloats(laserSensorOrientation));
        
        % reshape from 1 x 2052 vector to 
        % 3 rows (row x, row y, row z) x 684 columns matrix
        cartesianMat = reshape(pointCloudVec,3,[]);

        % 684 is ALL sampling points of the sensor. We need to filter
        % out only the useful ones
        % Convert to polar co-ordinates for easy filtering
        [theta, rho, z] = cart2pol(cartesianMat(1,:,:), cartesianMat(2,:,:), cartesianMat(3,:,:));
        polarPair = [theta; rho; z];
        % filter points between -120 to +120
        fov = sensor.FieldOfView;
        polarPair = polarPair(:, polarPair(1,:)>-fov/2 & polarPair(1,:)<fov/2);
        % filter points within min max range
        minRange = sensor.MinRange;
        maxRange = sensor.MaxRange;
        polarPair = polarPair(:, polarPair(2,:)>minRange & polarPair(2,:)<maxRange);
        
        % convert back to cartesian
        [cartX, cartY, cartZ] = pol2cart(polarPair(1,:), polarPair(2,:), polarPair(3,:));

        % Obtaining new co-ordinate points in global reference frame
        globalShifted = convertObjFrameToCsimGlobalFrame(...
            cartX, cartY, cartZ, hokuyoOrient, hokuyoOrigin);

        % VREP global frame origin is at center
        % MATLAB's occupancy map origin is at lower left corner
        % VREP's global frame origin with respect to MATLAB's origin is
        % at X/2, Y/2 => (2.5, 5)
        % with respect to MATLAB's occupancy map co-ordinate system
        % Add small correctional bias too
        % Note: Since we are building a 2-D map, Z value is discarded
        globalShifted = [...
            globalShifted(1,:) + maxXPos/2 + xAxisCorrectionFactor; ...
            globalShifted(2,:) + maxYPos/2 + yAxisCorrectionFactor];

        if ~isempty(globalShifted)
            % for now, hardcode 100% confidence for occupancy
            pValues = ones(1,length(globalShifted));
            % use updateOccupancy function for probabilistic "occupancyMap" object
            % for binary occupancy, we use setOccupancy function
            % finalWorld' is transpose of finalWorld
            setOccupancy(currOccMap,globalShifted',pValues);
        else
            disp('No valid points left after filtering');
        end
    else
        disp('No data');
    end
end