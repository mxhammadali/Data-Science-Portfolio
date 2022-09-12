% CoppeliaSim Lidar Sensor Class
classdef CsimLidarSensor
   properties
      MaxRange double
      MinRange double
      FieldOfView double
      PointCloudDataPointer char
      LaserOriginDataPointer char
      LaserOrientationDataPointer char
      Initialised logical = 0
   end
end