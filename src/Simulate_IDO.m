function [SPP_errors, SPP_falsePositiveErrors, SPP_falseNegativeErrors, elapsedTimes] = Simulate_IDO(max_vel,decay_factor,obstacles,figures_t,sim_horizon,n_run)
    %% Main parameters
    %sim_horizon = 80;
    lidar_range = 20;
    width = 50;
    height = 50;
    resolution = 2;
    n_obstacles = size(obstacles,1);

    %% Define the ego trajectory
    x = 25*ones(sim_horizon,1);
    y = 25*ones(sim_horizon,1);
    z = zeros(sim_horizon,1);
    X = [x y z];
    
    %% Define the scenario
    Scenarios = cell(1,sim_horizon);
    for i = 1:sim_horizon
        scenario = binaryOccupancyMap(width,height,resolution);
        for j = 1:n_obstacles
            scenario.setOccupancy(obstacles(j,:,i),1)
        end
        scenario.inflate(1)
        Scenarios{i} = scenario;
    end
    
    %% Create a lidar sensor
    sensor = lidar(lidar_range,0.01,0.01);
    
    %% Create a tracker
    prior = 0.5;
    range = lidar_range;
    invModel = [0.1 0.9];
    
    tracker = Tracker(prior,width,height,resolution,range,invModel,max_vel,decay_factor);
    
    %% Main loop
    % Struct to store the video frames
    Frames = struct('cdata', cell(1,sim_horizon), 'colormap', cell(1,sim_horizon));
    
    % Vector to store elapsed time
    elapsedTimes = zeros(sim_horizon,1);
    
    % Vectors to store errors
    SPP_errors = zeros(sim_horizon,1);
    SPP_falsePositiveErrors = zeros(sim_horizon,1);
    SPP_falseNegativeErrors = zeros(sim_horizon,1);
    
    for i = 1:sim_horizon
        % Update your possition
        x_t = X(i,:);
        % Update the scenario
        scenario_t = Scenarios{i};
        % Get the lidar reading
        [ranges, angles] = sensor.measure(x_t, scenario_t);
        z_t = lidarScan(ranges, angles);
        z_t = format_lidar_SPP(z_t, lidar_range);
        % Update the tracker
        tic
        tracker = tracker.update(z_t,x_t);
        elapsedTimes(i) = toc;
        % Compute the error
        [SPP_errors(i), SPP_falsePositiveErrors(i), SPP_falseNegativeErrors(i)] = computeError(scenario_t.getOccupancy, tracker.accMap.getOccupancy, x_t(1)*resolution, x_t(2)*resolution, lidar_range*resolution);
        % Video frame
        tracker.plot
        hold on
        plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
        title('Proposed method')
        Frames(i) = getframe(gcf);
        hold off
        % Figures
        if any(i == figures_t)
            % Ground Truth figure
            scenario_t.show()
            hold on
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            for j = 1:n_obstacles
                plot(permute(obstacles(j,1,1:i-2),[3,2,1]),permute(obstacles(j,2,1:i-2),[3,2,1]),'--k')
            end
            title('')
            xlabel('')
            ylabel('')
            set(gca,'XTick',[0,10,20,30,40], 'YTick', [0,10,20,30,40])
            saveas(gcf,strcat('../figures/Run_', int2str(n_run), '_T_', int2str(i), '_GT.svg'))
            hold off
            % Perception figure
            tracker.plot
            hold on
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            title('')
            xlabel('')
            ylabel('')
            set(gca,'XTick',[0,10,20,30,40], 'YTick', [0,10,20,30,40])
            saveas(gcf,strcat('../figures/Run_', int2str(n_run), '_T_', int2str(i),'_Method_IDO_Decay_',int2str(decay_factor*100),'.svg'))
            hold off
        end
    end
    
    %% Save the video
    VW = VideoWriter(strcat('../videos/Run_', int2str(n_run),'_Method_IDO_Decay_',int2str(decay_factor*100),'.svg'));
    VW.FrameRate = 10;
    VW.open();
    for Frame = Frames
        VW.writeVideo(Frame)
    end
    VW.close();
end