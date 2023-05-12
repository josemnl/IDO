function Scenario1_GM(lidar_range, width, height, resolution, images)
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
    prior = 0.5;
    range = lidar_range;
    invModel = [0.1 0.9];
    
    tracker = Tracker_GM(prior,width,height,resolution,range,invModel);
    
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
        title('Clamped Occ. Grid Map')
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
            tracker.plot
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
            tracker.plot
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
            tracker.plot
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
    VW = VideoWriter(strcat('../videos/Experiment1_GM.avi'));
    VW.FrameRate = 10;
    VW.open();
    for Frame = Frames
        VW.writeVideo(Frame)
    end
    VW.close();
end