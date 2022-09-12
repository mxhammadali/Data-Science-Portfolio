clear global
close all
clc

% The floor boundary in CoppeliaSim scene
X_BOUND = 5;
Y_BOUND = 10;
% Create binary occupancy map object
% Note: probabilistic "occupancyMap" object could also be used
currOccMap = binaryOccupancyMap(5,10,16);
occMapFigure = figure;
% For some reason, there is minor misalignment between VREP frame axes
% and the one being rendered in MATLAB occupancy map. Hence putting a
% minor correction factor for both X and Y axes
X_BIAS = 3.0;
Y_BIAS = -0.0;
lSpeed = 0;
rSpeed = 0;
csim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
csim.simxFinish(-1); % just in case, close all opened connections

clientID = csim.simxStart('192.168.1.133',19997,true,true,5000,5);
sim = remApi('remoteApi');
return_code = csim.simxStartSimulation(clientID,csim.simx_opmode_oneshot)

% Laser Scanner constants
% https://www.robotshop.com/media/files/pdf/hokuyo-urg-04lx-ug01-specifications.pdf
sensor = CsimLidarSensor;
sensor.MaxRange = 4.095; % in metres
sensor.MinRange = 0.06; % in metres
sensor.FieldOfView = 240 * pi/180; % 240 degrees in radians
sensor.PointCloudDataPointer = 'laserRangeFinderData';
sensor.LaserOriginDataPointer = 'laserOriginData';
sensor.LaserOrientationDataPointer = 'laserOrientationData';

% Driving constants
WHEEL_SPEED = 2;

if (clientID>-1)
    disp('Connected to CoppeliaSim');

    % Your code here
    
    % get object handles for the wheels
    [~,leftMotor]=csim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',csim.simx_opmode_blocking);
    [~,rightMotor]=csim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',csim.simx_opmode_blocking);
    [res,objs]=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);
    [res, leftMotor] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
    [res, rightMotor] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity(clientID,leftMotor,lSpeed,sim.simx_opmode_blocking);
    sim.simxSetJointTargetVelocity(clientID,rightMotor,rSpeed,sim.simx_opmode_blocking);
     
    [errorCode, fLeft] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor4',sim.simx_opmode_oneshot_wait);
    [errorCode, fRight] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5',sim.simx_opmode_oneshot_wait);
    [errorCode, sLeft] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor2',sim.simx_opmode_oneshot_wait);
    [errorCode, sRight] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor7',sim.simx_opmode_oneshot_wait);
     
    [errorCode, detectionState1, detectedPoint1, detectedObjectHandle, detectedSurfaceNormalVector] = sim.simxReadProximitySensor(clientID, fLeft, sim.simx_opmode_streaming);
    [errorCode, detectionState1, detectedPoint2, detectedObjectHandle, detectedSurfaceNormalVector] = sim.simxReadProximitySensor(clientID, fRight, sim.simx_opmode_streaming);
    [errorCode, detectionState1, detectedPoint3, detectedObjectHandle, detectedSurfaceNormalVector] = sim.simxReadProximitySensor(clientID, sLeft, sim.simx_opmode_streaming);
    [errorCode, detectionState1, detectedPoint4, detectedObjectHandle, detectedSurfaceNormalVector] = sim.simxReadProximitySensor(clientID, sRight, sim.simx_opmode_streaming);
  
    % We will use user input to navigate the robot
    userInput = 0;
    while userInput ~=27
        currOccMap = lidarFillOccMap(currOccMap,csim,clientID,sensor, ...
            X_BOUND, Y_BOUND, X_BIAS, Y_BIAS);
        figure(occMapFigure);
        show(currOccMap);

        % Before calling this function, please ensure that a "figure"
        % object is already present.
        % Also, you need to press ANYWHERE inside the figure object ONCE to
        % start listening for keystrokes
        userInput = 1;
        x = moveRobotFromProximity(csim,clientID,leftMotor,rightMotor,WHEEL_SPEED,fLeft,fRight,sLeft,sRight);
        disp(userInput);
    end

    % Your code ends
    csim.simxFinish(clientID);
else
    disp('Error connecting to CoppeliaSim');
end

csim.delete();

function userInput = moveRobotFromProximity(sim,clientID,...
    leftMotor,rightMotor,speed,fLeft,fRight,sLeft,sRight)
    [errorCode, detectionState1, detectedPoint1, detectedObjectHandle, detectedSurfaceNormalVector] = sim.simxReadProximitySensor(clientID, fLeft, sim.simx_opmode_streaming);
    [errorCode, detectionState1, detectedPoint2, detectedObjectHandle, detectedSurfaceNormalVector] = sim.simxReadProximitySensor(clientID, fRight, sim.simx_opmode_streaming);
    [errorCode, detectionState1, detectedPoint3, detectedObjectHandle, detectedSurfaceNormalVector] = sim.simxReadProximitySensor(clientID, sLeft, sim.simx_opmode_streaming);
    [errorCode, detectionState1, detectedPoint4, detectedObjectHandle, detectedSurfaceNormalVector] = sim.simxReadProximitySensor(clientID, sRight, sim.simx_opmode_streaming);
    userInput = 1;
    lSpeed = 0;
    rSpeed = 0;
    fDist = (detectedPoint1(2)+detectedPoint2(2))/2;
    
    if (fDist == 0)
        fDist = 1;
    end

    
    
    lDist = detectedPoint3(2);
    if (lDist == 0)
        lDist = 1;
    end
     rDist =detectedPoint4(2);
    if (rDist == 0)
        rDist = 1;
    end
    disp([fDist,lDist,rDist]);
    %disp(class(rDist));
    disp(class(0.00000001));
    
    
    if (rDist < 0.00000001) 
        if(detectedPoint2(2)<  0.00000001)
            lSpeed = -1;
            rSpeed = 1;
        else
            lSpeed = 0;
            rSpeed = 1;
        end
    end


    if (fDist > 0.00000001)
            lSpeed = 2;
            rSpeed = 2;
    end   
    if ( (fDist < 0.00000001)& (rDist < 0.00000001) & (lDist < 0.00000001))
        lSpeed = -2;
        rSpeed = 2;
    end
        
sim.simxSetJointTargetVelocity(clientID,leftMotor,lSpeed,sim.simx_opmode_blocking);
   sim.simxSetJointTargetVelocity(clientID,rightMotor,rSpeed,sim.simx_opmode_blocking);   
   
 
  

end