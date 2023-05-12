classdef lidar
    properties
        range
        resolution
        rangeSensor
        noiseFactor
    end
    
    methods
        function obj = lidar(range, resolution, noiseFactor)
            obj.range = range;
            obj.resolution = resolution;
            obj.rangeSensor = rangeSensor('Range',[0,range],'HorizontalAngleResolution',resolution);
            obj.noiseFactor = noiseFactor;
        end
        function [ranges, angles] = measure(obj, x_t, binaryMap)
            [ranges, angles] = obj.rangeSensor(x_t, binaryMap);
            ranges = ranges + obj.noiseFactor*rand(size(ranges));
        end
    end
end