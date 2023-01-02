function [elapsedTimes] = Scenario3_IDO(lidar_range, width, height, resolution, max_vel, images)
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
    prior = 0.5;
    range = lidar_range;
    invModel = [0.1 0.9];
    
    tracker = Tracker(prior,width,height,resolution,range,invModel, max_vel);
    
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
        z_t = lidarScan(ranges, angles);
        z_t = format_lidar_SPP(z_t, lidar_range);
        % Update the map
        tic
        tracker = tracker.update(z_t,x_t);
        elapsedTimes(i) = toc;
        % Video frame
        tracker.plot
        hold on
        plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
        title('Proposed method')
        Frames(i) = getframe(gcf);
        hold off
        % Figures
        if i == 94
            % Figure 1
            tracker.plot
            hold on
            plot(x_t(1),x_t(2),'ro','MarkerFaceColor','red')
            title('Proposed method')
            saveas(gcf,strcat('../figures/Experiment3.', images(1), '.svg'))
            hold off
        end
    end
    
    %% Save the video
    VW = VideoWriter(strcat('../videos/Scenario3_IDO_',string(max_vel),'.avi'));
    VW.FrameRate = 10;
    VW.open();
    for Frame = Frames
        VW.writeVideo(Frame)
    end
    VW.close();
end