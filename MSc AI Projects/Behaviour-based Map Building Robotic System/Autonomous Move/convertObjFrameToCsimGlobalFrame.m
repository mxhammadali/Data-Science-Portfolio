% function to convert sensor points from the sensor's co-ordinate frame to
% CoppeliaSim global reference frame
% INPUT: Row Matrix of X, Y, Z matrices, Euler Angles of Sensor Frame,
% Origin (translation shift) of sensor frame with respect to CoppeliaSim
% global reference frame
function globalShifted = convertObjFrameToCsimGlobalFrame(...
    cartX, cartY, cartZ, eulerAngles, translation)
    % Build a rotation matrix since laser axes have relative pose /
    % orientation with respect to global origin frame. Also,
    % remember that all point cloud coordinates are relative to
    % laser frame
    % Convert Euler Angles to Rotation Matrix
    % The Euler angles are specified in the axis rotation sequence. 
    % The default order for Euler angle rotations is "ZYX".
    % https://uk.mathworks.com/help/robotics/ref/eul2rotm.html
    % CoppeliaSim also follows the same order for Euler Angles
    % https://www.coppeliarobotics.com/helpFiles/en/eulerAngles.htm
    % More reading
    % https://uk.mathworks.com/help/robotics/ug/coordinate-transformations-in-robotics.html
    % https://uk.mathworks.com/help/fusion/ug/rotations-orientation-and-quaternions.html
    rotationMatrix = eul2rotm([0,0,eulerAngles(1)]) * ...
        eul2rotm([0,eulerAngles(2),0]) * ...
        eul2rotm([eulerAngles(3),0,0]);

    % Note for above: Out of the 3 euler angles, Rz(γ) - also known as Yaw
    % angle - is the most significant one. This is angle which signifies
    % rotation about Z axis (aka, rotation in the XY plane). We know from
    % the robot movement in CoppeliaSim that all motion happens in XY plane
    % only. Hence, euler angle values for Rx and Ry for laser
    % reference frame rotation would be 0 or near 0.
    % 
    % However, for the sake of completeness and generality, we have kept
    % the rot_mat (rotation matrix) as a product of all 3 rotations (Rx, Ry
    % and Rz)

    % We want to inverse the XYZ points rotation, so we multiply the
    % current XYZ points with the inverse of the matrix
    % A / B is faster syntax for A * inv(B)
    % cartX' is transpose of cart
    globalRotated = [cartX', cartY', cartZ'] / rotationMatrix;
    % globalRotated' is transpose of globalRotated
    globalRotated = globalRotated';
    
    % translational shift from laser frame to global origin frame
    % https://doubleroot.in/lessons/coordinate-geometry-basics/translation-of-axes/
    globalShifted = [...
        globalRotated(1,:) + translation(1); ...
        globalRotated(2,:) + translation(2); ...
        globalRotated(3,:) + translation(3)];
end