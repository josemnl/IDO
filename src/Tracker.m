classdef Tracker
    properties
        prior
        width
        height
        resolution
        range
        invModel
        max_vel
        convShape
        accMap
        decay_factor
    end

    methods
        function obj = Tracker(prior, width, height, resolution, range, ...
                              invModel, max_vel, decay_factor)
            obj.prior = prior;
            obj.width = width;
            obj.height = height;
            obj.resolution = resolution;
            obj.range = range;
            obj.invModel = invModel;
            obj.max_vel = max_vel;
            r = ceil(max_vel*resolution);
            shape = strel("disk",r).Neighborhood;
            obj.convShape = shape./sum(shape,'all');
            obj.accMap = occupancyMap(prior* ...
                ones(height*resolution, width*resolution),resolution);
            obj.decay_factor = decay_factor;
        end

        function plot(obj)
            obj.accMap.show;
        end

        function obj = update(obj, z_t, x_t)
            instantMap = instantMapper(obj, z_t, x_t);
            instMatrix = instantMap.occupancyMatrix;
            instOdds = instMatrix ./ (1-instMatrix);
            prediction = obj.predict();
            predMatrix = prediction.occupancyMatrix;
            predOdds = predMatrix ./ (1-predMatrix);
            priorOdds = obj.prior/(1-obj.prior);
            postOdds = instOdds .* (predOdds.^obj.decay_factor) / (priorOdds.^obj.decay_factor);
            postMatrix = postOdds ./ (1+postOdds);
            obj.accMap = occupancyMap(postMatrix,obj.resolution);
        end

        function obj = onlypredict(obj)
            obj.accMap = obj.predict();
        end

        function obj = forget(obj, z_t, x_t)
            obj.accMap = instantMapper(obj, z_t, x_t);
        end

        function instant_map = instantMapper(obj, z_t, x_t)
            % Create an empty map with the right dimentions
            instant_map = occupancyMap(obj.prior*ones(obj.height* ...
                obj.resolution, obj.width*obj.resolution),obj.resolution);
            % Populate the map based on the sensor's detections
            instant_map.insertRay(x_t, z_t, obj.range,obj.invModel);
        end

        function prediction = predict(obj)
            % Substract and add back the prior to fix the zero-padded edges
            map = obj.accMap.occupancyMatrix - obj.prior;
            prediction = conv2(map,obj.convShape,'same');
            prediction = prediction + obj.prior;
            prediction = occupancyMap(prediction,obj.resolution);
        end
    end
end