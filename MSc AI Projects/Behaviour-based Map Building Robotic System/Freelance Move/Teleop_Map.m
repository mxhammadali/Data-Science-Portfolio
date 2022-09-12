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

csim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
csim.simxFinish(-1); % just in case, close all opened connections
clientID = csim.simxStart('192.168.1.133',19997,true,true,5000,5);
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
    opMode = csim.simx_opmode_blocking;       
    csim.simxSetJointTargetVelocity(clientID,leftMotor,0,opMode);
        csim.simxSetJointTargetVelocity(clientID,rightMotor,0,opMode);

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
        userInput = moveRobotFromUserInput(csim,clientID,leftMotor,rightMotor,WHEEL_SPEED);
        disp(userInput);
    end

    % Your code ends
    csim.simxFinish(clientID);
else
    disp('Error connecting to CoppeliaSim');
end

csim.delete();

function userInput = moveRobotFromUserInput(csim,connId,...
    leftMotor,rightMotor,speed)
    % solicit keyboard press
    disp('Waiting for user input');
    userInput=waitforbuttonpress;
    
    % keyboard press key
    % source: https://www.youtube.com/watch?v=-hT5htewFEs
    % up arrow -> move forward
    % down arrow -> move backwards
    % left arrow -> move left
    % right arrow -> move right
    % 0 key -> pause
    % esc key -> terminate
    val=double(get(gcf,'CurrentCharacter'));
    opMode = csim.simx_opmode_blocking;
    if val==30 %forward
        csim.simxSetJointTargetVelocity(connId,leftMotor,speed,opMode);
        csim.simxSetJointTargetVelocity(connId,rightMotor,speed,opMode);
    elseif val==28 %left
        csim.simxSetJointTargetVelocity(connId,leftMotor,0,opMode);
        csim.simxSetJointTargetVelocity(connId,rightMotor,speed,opMode);
    elseif val==29 %right
        csim.simxSetJointTargetVelocity(connId,leftMotor,speed,opMode);
        csim.simxSetJointTargetVelocity(connId,rightMotor,0,opMode);
    elseif val==31 %backwards
        csim.simxSetJointTargetVelocity(connId,leftMotor,-1*speed,opMode);
        csim.simxSetJointTargetVelocity(connId,rightMotor,-1*speed,opMode);
    elseif val==48 %pause
        csim.simxSetJointTargetVelocity(connId,leftMotor,0,opMode);
        csim.simxSetJointTargetVelocity(connId,rightMotor,0,opMode);
    end %note that pressing the esc key will exit the while loop
end