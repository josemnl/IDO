function Scenario1_GM(lidar_range, width, height, resolution, images)
    %% Define the ego trajectory
    v = 0.5;
    X = linearTrajectoryVel([6,32],[43,32],v);
    sim_horizon = size(X,1);
    ob1_X = linearTrajectoryVel([43,44],[6,44],v);
    ob2_X = linearTrajectoryVel([6,13],[43,13],v);
    
    %% Import the map
    image = imread('room.png');
    grayimage = rgb2gray(image);
    bwimage = grayimage < 0.5;
    room = binaryOccupancyMap(bwimage);
    room = binaryOccupancyMap(room,resolution);
    
    Scenarios = cell(1,sim_horizon);
    
    for i = 1:sim_horizon
        scenario = binaryOccupancyMap(room);
        scenario.setOccupancy(ob1_X(i,1:2)-[1,1],ones(4,4));
        scenario.setOccupancy(ob2_X(i,1:2)-[1,1],ones(4,4));
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
    
    for i = 1:sim_horizon
        % Update ego possition
        x_t = X(i,:);
        % Update the scenario
        scenario_t = Scenarios{i};
        % Get the lidar reading
        [ranges, angles] = sensor.measure(x_t, scenario_t);
        z_t = lidarScan(ranges, angles);
        z_t = format_lidar_SPP(z_t, lidar_range);
        % Update the tracker
        tracker = tracker.update(z_t,x_t);
        % Video frame
        tracker.plot
        hold on
        plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
        title('GM method')
        Frames(i) = getframe(gcf);
        hold off
        % Figures
        if i == 40
            % Figure 1
            scenario_t.show()
            hold on
            plot(X(1:i,1),X(1:i,2),'--r')
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            plot([ob1_X(1,1) ob1_X(i,1)],[ob1_X(1,2) ob1_X(i,2)],'--k')
            plot([ob2_X(1,1) ob2_X(i,1)],[ob2_X(1,2) ob2_X(i,2)],'--k')
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
        if i == 50
            % Figure 2
            scenario_t.show()
            hold on
            plot(X(1:i,1),X(1:i,2),'--r')
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            plot([ob1_X(1,1) ob1_X(i,1)],[ob1_X(1,2) ob1_X(i,2)],'--k')
            plot([ob2_X(1,1) ob2_X(i,1)],[ob2_X(1,2) ob2_X(i,2)],'--k')
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
        if i == sim_horizon
            % Figure 3
            scenario_t.show()
            hold on
            plot(X(1:i,1),X(1:i,2),'--r')
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            plot([ob1_X(1,1) ob1_X(i,1)],[ob1_X(1,2) ob1_X(i,2)],'--k')
            plot([ob2_X(1,1) ob2_X(i,1)],[ob2_X(1,2) ob2_X(i,2)],'--k')
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
    VW = VideoWriter('../videos/Scenario1_GM.avi');
    VW.FrameRate = 10;
    VW.open();
    for Frame = Frames
        VW.writeVideo(Frame)
    end
    VW.close();
end