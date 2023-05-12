classdef Tracker_GM
    properties
        prior
        width
        height
        resolution
        range
        invModel
        accMap
    end

    methods
        function obj = Tracker_GM(prior, width, height, resolution, range, invModel)
            obj.prior = prior;
            obj.width = width;
            obj.height = height;
            obj.resolution = resolution;
            obj.range = range;
            obj.invModel = invModel;
            obj.accMap = occupancyMap(prior* ...
                ones(height*resolution, width*resolution),resolution);
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
            postOdds = instOdds .* predOdds / priorOdds;
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
            prediction = obj.accMap;
        end
        
    end
end