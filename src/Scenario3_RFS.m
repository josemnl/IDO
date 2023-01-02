function [elapsedTimes] = Scenario3_RFS(lidar_range, width, height, resolution, FreeSpaceDiscountFactor, images)
    %% Import rosbags and format the lidar readings
    bag = rosbag('../rosbags/2022-09-20-13-56-37.bag');
    bSel = select(bag,'Topic','/scan');
    msgScans = readMessages(bSel,'DataFormat','object');
    sim_horizon = size(msgScans,1);
    
    %% Define the ego trajectory: x_t = x(t,:)
    x = 4*ones(sim_horizon,1);
    y = 4*ones(sim_horizon,1);
    z = zeros(sim_horizon,1);
    X = [x y z];
    
    %% Create a tracker
    config = trackingSensorConfiguration(1, ...
        'SensorLimits',[-180 180;0 lidar_range], ...
        'SensorTransformParameters',struct, ...
        'IsValidTime',true);
    
    tracker = trackerGridRFS('SensorConfigurations',config, ...
        'AssignmentThreshold',5, ...
        'GridLength',width, ...
        'GridWidth',height, ...
        'GridOriginInLocal', [0 0], ...
        'GridResolution',resolution, ...
        'MinNumCellsPerCluster',4, ...
        'VelocityLimits', [-1 1; -1 1], ...
        'ClusteringThreshold',3, ...
        'HasSensorConfigurationsInput',true, ...
        'FreeSpaceDiscountFactor', FreeSpaceDiscountFactor);
    
    %% Main loop
    % Struct to store the video frames
    Frames = struct('cdata', cell(1,sim_horizon), 'colormap', cell(1,sim_horizon));
    
    % Vector to store elapsed time
    elapsedTimes = zeros(sim_horizon,1);
    
    for i = 1:sim_horizon
        % Update your possition
        x_t = X(i,:);
        % Get the lidar reading
        angles = msgScans{i}.AngleMin:msgScans{i}.AngleIncrement:msgScans{i}.AngleMax;
        ranges = msgScans{i}.Ranges;
        % Remove lost rays
        rangesAngles = [ranges,angles'];
        rangesAngles(rangesAngles(:, 1)== 0, :)= [];
        z_t = lidarScan(rangesAngles(:,1), rangesAngles(:,2));
        % Format the lidar reading
        [sensorData, config] = format_lidar_RFS(z_t, x_t, i, lidar_range);
        % Update the tracker
        tic
        [tracks, ~, ~, map] = tracker(sensorData,{config},i);
        elapsedTimes(i) = toc;
        % Video frame
        grid = occupancyMap(map.getOccupancy,resolution);
        grid.show('local')
        hold on
        plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
        title('RFS method')
        Frames(i) = getframe(gcf);
        hold off
        % Figures
        if i == 94
            % Figure 1
            grid.show('local')
            hold on
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            title('RFS method')
            saveas(gcf,strcat('../figures/Experiment3.', images(1), '.svg'))
            hold off
        end
    end
    
    %% Save the video
    VW = VideoWriter(strcat('../videos/Scenario3_RFS_',string(FreeSpaceDiscountFactor),'.avi'));
    VW.FrameRate = 10;
    VW.open();
    for Frame = Frames
        VW.writeVideo(Frame)
    end
    VW.close();
end