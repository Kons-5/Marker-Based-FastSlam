clear; clf; clc; close all

%% Set figure size
set(gcf,"OuterPosition", [100 100 600 600]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The number of timesteps for the simulation
timesteps = 1000;

% The maximum distance from which our sensor can sense a landmark
max_read_distance = 1.8;

% The actual positions of the landmarks (each column is a separate landmark)
real_landmarks = [3.0000, 2.5981, 1.5000, 0.0000, -1.5000, -2.5981, -3.0000, -2.5981, -1.5000, 0.0000, 1.5000, 2.5981;   % x
                  0.0000, 1.5000, 2.5981, 3.0000, 2.5981, 1.5000, 0.0000, -1.5000, -2.5981, -3.0000, -2.5981, -1.5000;   % y
                  0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000];      % z


% The initial starting position of the robot
real_position = [ 0.0;      % x
                 -2.0;      % y
                  0.0];     % rotation

% The movement command given to the robot at each timestep                 
movement_command = [.04;     % Linear Velocity
                    .02  ];    % Angular Velocity
                    
% The Gaussian variance of the movement commands
movement_variance = [.03;    % Linear Velocity
                     .025];  % Angular Velocity

M = [movement_variance(1), 0.0;
     0.0, movement_variance(2)];

% The Gaussian variance of our sensor readings
measurement_variance = [0.1;    % Distance
                        0.01    % Angle
                       ]; 

R = [measurement_variance(1), 0.0;
     0.0, measurement_variance(2)];

% Create the particles and initialize them all to be in the same initial
% position. 
particles = [];

% Define the number of particles and timesteps
num_particles = 200; 

% Initialize particle structure array
particles = struct('w', [], 'position', [], 'lm_weight', [], 'num_landmarks', [], 'landmarks', [], "num_existing_landmarks", []);

% Pre-allocate the particles array
[particles(1:num_particles).w] = deal(1.0 / num_particles);
[particles(1:num_particles).position] = deal(real_position);
[particles(1:num_particles).lm_weight] = deal(1.0 / num_particles);
[particles(1:num_particles).num_existing_landmarks] = deal(0);

% Pre-allocate num_landmarks for all particles
empty_landmarks = zeros(1, timesteps);
for i = 1:num_particles
    particles(i).num_landmarks = empty_landmarks;
end

pos_history = [];           % Array of all of the robot's position
mean_trajectory = [];       % Array of particle mean trajectory
feature_to_delete = [];
default_importance = 1e-3; 
num_existing_landmarks = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

delta_t = 1; % Set sampling Time
for timestep = 2:delta_t:timesteps

    % Move the actual robot
    real_position = moveParticle(real_position, movement_command,[0;0], delta_t);
    pos_history = [pos_history, real_position];

    % Move the actual particles
    for pIdx = 1:num_particles
        particles(pIdx).position = moveParticle(particles(pIdx).position, movement_command, movement_variance, delta_t);
    end

    % Try to take a reading from each landmark
    doResample = false;
    
    % Take measurements
    z_real = [];
    G = [];
    read_distance = [];
    read_angle = [];
    
    for lIdx = 1:size(real_landmarks, 2)
        real_landmark = real_landmarks(:, lIdx);
        [z_real(:, lIdx), G(:, :, lIdx)] = getMeasurement(real_position, real_landmark, measurement_variance);
        read_distance(lIdx) = z_real(1, lIdx);
        read_angle(lIdx) = z_real(2, lIdx);
    end
    
    % Filter measurements within range
    to_keep = find(read_distance < max_read_distance);
    z_real = z_real(:, to_keep);
    read_angle = read_angle(to_keep);
    read_distance = read_distance(to_keep);
    G = G(:, :, to_keep);
    
    % Resample if there are new measurements
    if ~isempty(z_real)
        doResample = true; 
    end

    % Process each particle
    for pIdx = 1:num_particles
        % If there are no new landmarks skip
        if isempty(z_real)
            continue;
        end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Data Association
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    num_measurements = size(z_real, 2);
    num_existing_landmarks = particles(pIdx).num_existing_landmarks;
    cost_table = zeros(num_measurements, num_existing_landmarks + num_measurements);
    
    for j = 1:num_existing_landmarks
        [z_p(:, j), H(:, :, j)] = getMeasurement(particles(pIdx).position, particles(pIdx).landmarks(j).pos, [0; 0]);
        Q(:, :, j) = H(:, :, j) * particles(pIdx).landmarks(j).E * H(:, :, j)' + R;
    end
    
    % Compute cost table
    for measured_lm = 1:num_measurements
        for j = 1:num_existing_landmarks
            residual = z_real(:, measured_lm) - z_p(:, j);
            cost_table(measured_lm, j) = (1 / sqrt(det(2 * pi * Q(:, :, j)))) * exp(-0.5 * (residual' / Q(:, :, j) * residual));
        end
    end
    
    cost_table(:, num_existing_landmarks + 1:end) = default_importance * eye(num_measurements);
    
    % Initialize vectors for data association
    data_associate_vect = zeros(1, num_measurements);
    data_associate_vect_index = zeros(1, num_measurements);
    
    for measured_lm = 1:num_measurements
        % Find the best matching landmark for each measurement
        [val, best_landmark_idx] = max(cost_table(measured_lm, :));
        particles(pIdx).lm_weight = particles(pIdx).lm_weight * val;  % Update the particle's weight
    
        if best_landmark_idx > num_existing_landmarks
            % This is a new landmark
            new_landmark_idx = particles(pIdx).num_existing_landmarks + 1;
            particles(pIdx).num_landmarks(timestep) = particles(pIdx).num_landmarks(timestep) + 1;
            particles(pIdx).num_existing_landmarks = new_landmark_idx;
    
            data_associate_vect(measured_lm) = new_landmark_idx;
        else 
            % This is an existing landmark
            data_associate_vect(measured_lm) = best_landmark_idx;
        end
        data_associate_vect_index(measured_lm) = measured_lm;
    end
    
    % Update the particle's weight based on the combined likelihood of all measurements
    particles(pIdx).w = particles(pIdx).w * particles(pIdx).lm_weight;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update Step
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Update landmarks based on the data association
    for j = 1:num_measurements
        landmark_idx = data_associate_vect(j);
        if landmark_idx > 0
            measured_idx = data_associate_vect_index(j);
    
            if landmark_idx > particles(pIdx).num_landmarks(timestep-1)
                % Initialize new landmark
                particles(pIdx).landmarks(landmark_idx).pos = [
                    particles(pIdx).position(1) + cos(read_angle(measured_idx)) * read_distance(measured_idx);
                    particles(pIdx).position(2) + sin(read_angle(measured_idx)) * read_distance(measured_idx)];
    
                % Initialize the landmark position covariance
                H_new = G(:, :, measured_idx);
                particles(pIdx).landmarks(landmark_idx).E = inv(H_new) * R * inv(H_new)';
                particles(pIdx).landmarks(landmark_idx).counter = 1; % Initialize counter
            else
                % Update existing landmark
                % Calculate the Kalman gain
                Q_ = H(:, :, j) * particles(pIdx).landmarks(landmark_idx).E * H(:, :, j)' + R;
                K = particles(pIdx).landmarks(landmark_idx).E * H(:, :, j)' / Q_;
    
                % Update the landmark mean with the new measurement
                particles(pIdx).landmarks(landmark_idx).pos = particles(pIdx).landmarks(landmark_idx).pos + K * (z_real(:, measured_idx) - z_p(:, j));
    
                % Update the covariance of this landmark
                particles(pIdx).landmarks(landmark_idx).E = (eye(size(K, 1)) - K * H(:, :, j)) * particles(pIdx).landmarks(landmark_idx).E;
                particles(pIdx).landmarks(landmark_idx).counter = particles(pIdx).landmarks(landmark_idx).counter + 1; % Increment counter
            end
        end
    end
end

    % Resample all particles based on their weights
    if(doResample)
     particles = resample(particles);
    end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % PLOTTING
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  clf;
  hold on;

  % Plot the real landmarks
  for lIdx=1:size(real_landmarks,2)
      plot(real_landmarks(1,lIdx), real_landmarks(2,lIdx), 'b*');
  end
    
  %     % Plot all particle estimates of landmarks
  % for pIdx = 1:length(particles)
  %     for lIdx=1:length(particles(pIdx).landmarks)
  %         plot(particles(pIdx).landmarks(lIdx).pos(1),particles(pIdx).landmarks(lIdx).pos(2),'g.')
  %     end
  % end

  

  % Plot the particles
  particles_pos = [particles.position];
  plot(particles_pos(1,:), particles_pos(2,:), 'r.');
  
  % Plot the real robot
  plot(pos_history(1,:), pos_history(2,:), 'r');
  w = .1;
  l = .3;
  x = real_position(1);
  y = real_position(2);
  t = real_position(3);
  plot(real_position(1), real_position(2), 'mo', ...
                                           'LineWidth',1.5, ...
                                           'MarkerEdgeColor','k', ...
                                           'MarkerFaceColor',[0 1 0], ...
                                           'MarkerSize',5);
                                       
   
  % Show the sensor measurement as an arrow
  for lIdx=1:size(real_landmarks,2)
    real_landmark = real_landmarks(:, lIdx);
      try
          line([real_position(1), real_position(1)+cos(read_angle(lIdx))*read_distance(lIdx)], ...
           [real_position(2), real_position(2)+sin(read_angle(lIdx))*read_distance(lIdx)], 'LineStyle','--');
        end
  end

  axis([-5, 5, -5, 5]);
  grid minor;
  pause(.001);
end