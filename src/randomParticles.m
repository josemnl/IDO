function positions = randomParticles(num_particles, r_outer, r_inner, particle_size, dt, t_end, max_vel, x_0, y_0)
    % Define parameters
    %num_particles = 10;
    %r = 5; % radius of circle boundary
    
    %particle_size = 0.5;
    
    % Set simulation parameters
    %dt = 0.1; % time step
    %t_end = 10; % simulation time
    
    % Initialize positions and velocities
    positions = zeros(num_particles, 2, t_end/dt);
    velocities = rand(num_particles, 2)*max_vel;
    
    % Generate initial positions within circle boundary
    for i = 1:num_particles
        while true
            x = (rand() * 2 - 1) * r_outer; % random x coordinate within [-r, r]
            y = (rand() * 2 - 1) * r_outer; % random y coordinate within [-r, r]
            if norm([x, y]) >= r_inner && norm([x, y]) <= r_outer % check if point is inside circle
                positions(i, :, 1) = [x, y];
                break;
            end
        end
    end
    
    % Plot initial state
    figure;
    scatter(positions(:,1,1), positions(:,2,1), 'filled');
    hold on;
    rectangle('Position',[-r_outer -r_outer 2*r_outer 2*r_outer],'Curvature',[1 1],'EdgeColor','r');
    xlim([-r_outer r_outer]);
    ylim([-r_outer r_outer]);
    drawnow;
    
    % Simulation loop
    for t_step = 2:(t_end/dt)
        % Clear figure
        clf;
        
        % Update positions
        positions(:,:,t_step) = positions(:,:,t_step-1) + dt * velocities;
        
        % Check for collisions with other particles
        for i = 1:num_particles
            for j = i+1:num_particles
                r_ij = positions(i,:,t_step) - positions(j,:,t_step); % vector from particle j to particle i
                dist_ij = norm(r_ij);
                if dist_ij < particle_size % collision threshold
                    v_i = velocities(i,:);
                    v_j = velocities(j,:);
                    dot_ij = dot(r_ij, v_i-v_j);
                    if dot_ij < 0 % particles are moving towards each other
                        v_i_new = v_j;
                        v_j_new = v_i;
                        velocities(i,:) = v_i_new;
                        velocities(j,:) = v_j_new;
                    end
                end
            end
        end
        
        % Check for collisions with outer circle boundary
        for i = 1:num_particles
            if norm(positions(i,:,t_step)) > r_outer % particle is outside circle
                v_i = velocities(i,:);
                n_i = positions(i,:,t_step) / norm(positions(i,:,t_step)); % unit normal vector pointing towards origin
                v_i_new = v_i - 2 * dot(v_i, n_i) * n_i; % reflect velocity across circle boundary
                velocities(i,:) = v_i_new;
                positions(i,:,t_step) = r_outer * n_i; % set particle position to nearest point on circle
            end
        end

        % Check for collisions with inner circle boundary
        for i = 1:num_particles
            if norm(positions(i,:,t_step)) < r_inner % particle is inside inner circle
                v_i = velocities(i,:);
                n_i = -positions(i,:,t_step) / norm(positions(i,:,t_step)); % unit normal vector pointing away from origin
                v_i_new = v_i - 2 * dot(v_i, n_i) * n_i; % reflect velocity across circle boundary
                velocities(i,:) = v_i_new;
                positions(i,:,t_step) = -r_inner * n_i; % set particle position to nearest point on circle
            end
        end
        
        % Plot positions
        scatter(positions(:,1,t_step), positions(:,2,t_step), 'filled');
        rectangle('Position',[-r_outer -r_outer 2*r_outer 2*r_outer],'Curvature',[1 1],'EdgeColor','r');
        rectangle('Position',[-r_inner -r_inner 2*r_inner 2*r_inner],'Curvature',[1 1],'EdgeColor','r');
        xlim([-r_outer r_outer]);
        ylim([-r_outer r_outer]);
        drawnow;
    end
    positions(:,1,:) = positions(:,1,:) + x_0;
    positions(:,2,:) = positions(:,2,:) + y_0;
end
