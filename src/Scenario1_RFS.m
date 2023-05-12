function [RFS_errors, RFS_falsePositiveErrors, RFS_falseNegativeErrors] = Scenario1_RFS(lidar_range, width, height, resolution, FreeSpaceDiscountFactor, images)
    if FreeSpaceDiscountFactor == 1
        factor = 0.99999;
    else
        factor = FreeSpaceDiscountFactor;
    end
    %% Define the ego trajectory
    sim_horizon = 80;
    x = 25*ones(sim_horizon,1);
    y = 25*ones(sim_horizon,1);
    z = zeros(sim_horizon,1);
    X = [x y z];
    
    %% Define the scenario
    Scenarios = cell(1,sim_horizon);
    ob1_x0 = [5 20];
    ob1_v = [0.5 0];
    ob2_x0 = [45 10];
    ob2_v = [-0.5 0];
    for i = 1:sim_horizon
        scenario = binaryOccupancyMap(width,height,resolution);
        scenario.setOccupancy(ob1_x0+i*ob1_v,1);
        scenario.setOccupancy(ob2_x0+i*ob2_v,1);
        scenario.inflate(1)
        Scenarios{i} = scenario;
    end
    
    %% Create a lidar sensor
    sensor = lidar(lidar_range,0.01,0.01);
    
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
        'VelocityLimits', [-0.5 0.5; -0.5 0.5], ...
        'ClusteringThreshold',3, ...
        'HasSensorConfigurationsInput',true, ...
        'FreeSpaceDiscountFactor', factor);
    
    %% Main loop
    % Struct to store the video frames
    Frames = struct('cdata', cell(1,sim_horizon), 'colormap', cell(1,sim_horizon));
    
    % Vector to store elapsed time
    elapsedTimes = zeros(sim_horizon,1);
    
    % Vector to store error
    RFS_errors = zeros(sim_horizon,1);
    RFS_falsePositiveErrors = zeros(sim_horizon,1);
    RFS_falseNegativeErrors = zeros(sim_horizon,1);
    
    for i = 1:sim_horizon
        % Update your possition
        x_t = X(i,:);
        % Update the scenario
        scenario_t = Scenarios{i};
        % Get the lidar reading
        [ranges, angles] = sensor.measure(x_t, scenario_t);
        z_t = lidarScan(ranges, angles);
        % Format the lidar reading
        [sensorData, config] = format_lidar_RFS(z_t, x_t, i, lidar_range);
        % Update the tracker
        tic
        [tracks, ~, ~, map] = tracker(sensorData,{config},i);
        elapsedTimes(i) = toc;
        % Compute the error
        [RFS_errors(i), RFS_falsePositiveErrors(i), RFS_falseNegativeErrors(i)] = computeError(scenario_t.getOccupancy, map.getOccupancy, x_t(1)*resolution, x_t(2)*resolution, lidar_range*resolution);
        % Video frame
        grid = occupancyMap(map.getOccupancy,resolution);
        grid.show('local')
        title('RFS method')
        hold on
        plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
        Frames(i) = getframe(gcf);
        hold off
        % Figures
        if i == 35
            % Figure 1
            scenario_t.show()
            hold on
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            plot([ob1_x0(1) ob1_x0(1)+ob1_v(1)*i-2],[ob1_x0(2) ob1_x0(2)+ob1_v(2)*i],'--k')
            plot([ob2_x0(1) ob2_x0(1)+ob2_v(1)*i+2],[ob2_x0(2) ob2_x0(2)+ob2_v(2)*i],'--k')
            title('')
            xlabel('')
            ylabel('')
            set(gca,'XTick',[0,10,20,30,40], 'YTick', [0,10,20,30,40])
            saveas(gcf,strcat('../figures/Experiment1.', images(1), '.svg'))
            hold off
            % Figure 4
            grid.show('local')
            hold on
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            title('')
            xlabel('')
            ylabel('')
            set(gca,'XTick',[0,10,20,30,40], 'YTick', [0,10,20,30,40])
            saveas(gcf,strcat('../figures/Experiment1.', images(4), '.svg'))
            hold off
        end
        if i == 40
            % Figure 2
            scenario_t.show()
            hold on
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            plot([ob1_x0(1) ob1_x0(1)+ob1_v(1)*i-2],[ob1_x0(2) ob1_x0(2)+ob1_v(2)*i],'--k')
            plot([ob2_x0(1) ob2_x0(1)+ob2_v(1)*i+2],[ob2_x0(2) ob2_x0(2)+ob2_v(2)*i],'--k')
            title('')
            xlabel('')
            ylabel('')
            set(gca,'XTick',[0,10,20,30,40], 'YTick', [0,10,20,30,40])
            saveas(gcf,strcat('../figures/Experiment1.', images(2), '.svg'))
            hold off
            % Figure 5
            grid.show('local')
            hold on
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            title('')
            xlabel('')
            ylabel('')
            set(gca,'XTick',[0,10,20,30,40], 'YTick', [0,10,20,30,40])
            saveas(gcf,strcat('../figures/Experiment1.', images(5), '.svg'))
            hold off
        end
        if i == 45
            % Figure 3
            scenario_t.show()
            hold on
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            plot([ob1_x0(1) ob1_x0(1)+ob1_v(1)*i-2],[ob1_x0(2) ob1_x0(2)+ob1_v(2)*i],'--k')
            plot([ob2_x0(1) ob2_x0(1)+ob2_v(1)*i+2],[ob2_x0(2) ob2_x0(2)+ob2_v(2)*i],'--k')
            title('')
            xlabel('')
            ylabel('')
            set(gca,'XTick',[0,10,20,30,40], 'YTick', [0,10,20,30,40])
            saveas(gcf,strcat('../figures/Experiment1.', images(3), '.svg'))
            hold off
            % Figure 6
            grid.show('local')
            hold on
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            title('')
            xlabel('')
            ylabel('')
            set(gca,'XTick',[0,10,20,30,40], 'YTick', [0,10,20,30,40])
            saveas(gcf,strcat('../figures/Experiment1.', images(6), '.svg'))
            hold off
        end
    end
    
    %% Save the video
    VW = VideoWriter(strcat('../videos/Experiment1_RFS_',string(100*FreeSpaceDiscountFactor),'.avi'));
    VW.FrameRate = 10;
    VW.open();
    for Frame = Frames
        VW.writeVideo(Frame)
    end
    VW.close();
end