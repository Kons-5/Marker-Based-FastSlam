clear; clf; clc; close all

%% Set figure size
set(gcf,"OuterPosition", [100 100 600 600]);

%% PARAMETERS

% The number of timesteps for the simulation
timesteps = 1000;

% The maximum distance from which our sensor can sense a landmark
max_read_distance = 1.8;

% The actual positions of the landmarks (each column is a separate landmark)
real_landmarks = [-3.0, -3.0, 2.0, 0.0, 0.0, 1.0, 0.0, -1.0, 1.5, 0.5;   % x
                   2.0,  2.0, 2.5, 3.4, 1.5, 3.5,-3.0,  2.0, 0.0, 4.0;   % y
                   0.0,  1.0, 1.0, 0.5, 1.0, 1.0, 1.0,  1.0, 1.0, 1.0];  % z


% The initial starting position of the robot
real_position = [ 0.0;      % x
                 -2.0;      % y
                  0.0;      % z
                  0.0];  % rotation

% The movement command given to the robot at each timestep                 
movement_command = [.025;     % Linear Velocity
                    .01];    % Angular Velocity
                    
% The Gaussian variance of the movement commands
movement_variance = [.03;   % Linear Velocity
                     .025];  % Angular Velocity

M = [movement_variance(1), 0.0;
     0.0, movement_variance(2)];

% The Gaussian variance of our sensor readings
measurement_variance = [0.1;    % Distance
                        0.01;   % Azimuth
                        0.01;   % Elevation
                       ]; 

R = [measurement_variance(1), 0.0,      0.0;
     0.0, measurement_variance(2),      0.0;
     0.0,    0.0,       measurement_variance(3);
     ];

% Create the particles and initialize them all to be in the same initial
% position. 
particles = [];
num_particles = 100;
for i = 1:num_particles

  % Particles all start with the same normalized weight
  particles(i).w = 1.0/num_particles;
  particles(i).position = real_position;

  for lIdx=1:size(real_landmarks,2)
    particles(i).landmarks(lIdx).seen = false;
  end
end

pos_history = [];     % Array of all of the robot's position
mean_trajectory = []; % Array of particle mean trajectory

%% SIMULATION

delta_t = 2; % Set sampling Time
for timestep = 1:delta_t:timesteps

  % Move the actual robot
  real_position = moveParticle(real_position, movement_command,[0;0], delta_t);
  pos_history = [pos_history, real_position];

  % Move the actual particles
  for pIdx = 1:num_particles
    particles(pIdx).position = moveParticle( ...
        particles(pIdx).position, movement_command, movement_variance, delta_t);
  end

  % Try to take a reading from each landmark
  doResample = false;
  for lIdx = 1:size(real_landmarks,2)
    real_landmark = real_landmarks(:, lIdx);

    % Take a real (noisy) measurement from the robot to the landmark
    [z_real, G, p] = getMeasurement(real_position, real_landmark, measurement_variance);
    read_distance(lIdx) = z_real(1);
    read_azimuth(lIdx)  = z_real(2);
    read_elevation(lIdx)  = z_real(3);

    % If the landmark is close enough, then we can spot it
    if(read_distance(lIdx) < max_read_distance)
      doResample = true;

      for pIdx = 1:num_particles

        if(particles(pIdx).landmarks(lIdx).seen == false)

          % If we have never seen this landmark, then we need to initialize it.
          % We'll just use whatever first reading we received.
            particles(pIdx).landmarks(lIdx).pos = [
                particles(pIdx).position(1) + read_distance(lIdx) * cos(read_elevation(lIdx)) * cos(read_azimuth(lIdx));  % X coordinate
                particles(pIdx).position(2) + read_distance(lIdx) * cos(read_elevation(lIdx)) * sin(read_azimuth(lIdx));  % Y coordinate
                particles(pIdx).position(3) + read_distance(lIdx) * sin(read_elevation(lIdx))                             % Z coordinate
            ];

          % Initialize the landmark position covariance
          particles(pIdx).landmarks(lIdx).E = inv(G) * R * inv(G)';

          particles(pIdx).landmarks(lIdx).seen = true;

        else
          % Get previous learned position
          [z_p, Gp] = getMeasurement(particles(pIdx).position, particles(pIdx).landmarks(lIdx).pos, [0;0;0]);
          delta_z = z_real - z_p;

          %Calculate the Kalman gain
          Q = G * particles(pIdx).landmarks(lIdx).E * G' + R;
          K = particles(pIdx).landmarks(lIdx).E * G' * inv(Q);

          % Mix the previous reading, and our new reading using the Kalman gain, and use the result
          % to predict a new landmark position
          % Update the landmark mean with the new measurement
          particles(pIdx).landmarks(lIdx).pos = particles(pIdx).landmarks(lIdx).pos + K*(delta_z); 

          % Update the covariance of this landmark
          particles(pIdx).landmarks(lIdx).E = (eye(size(K)) - K*G)*particles(pIdx).landmarks(lIdx).E;

          % Update the weight of the particle
          particles(pIdx).w = particles(pIdx).w * norm(2*pi*Q).^(-1/2)*exp(-1/2*(delta_z)'*inv(Q)*(delta_z));
        end 
      end 
    end 
  end 

  % After updating particles' positions update mean trajectory
  particle_positions = reshape([particles.position], 4, num_particles); 
  mean_position = mean(particle_positions, 2);
  mean_trajectory = [mean_trajectory, mean_position];

  % Resample all particles based on their weights
  if(doResample)
    particles = resample(particles);
  end

  %% PLOTTING

  clf;
  hold on; view(70,30);
  
  % Plot the landmarks
  for lIdx=1:size(real_landmarks,2)
    plot3(real_landmarks(1,lIdx), real_landmarks(2,lIdx), real_landmarks(3,lIdx), "Marker","pentagram", "MarkerSize",10,"Color","b", "MarkerFaceColor","b");
  end

  % Loop over each landmark
  for lIdx = 1:size(real_landmarks, 2)
      % Check if the landmark has been seen by at least the first particle (assuming similar for others)
      if particles(1).landmarks(lIdx).seen

          % Initialize the average landmark guess for the current landmark
          avg_landmark_guess = zeros(3, 1);  % Use zeros for initialization
          avg_covariance = zeros(3, 3);

          % Sum positions of this landmark from all particles
          for pIdx = 1:length(particles)
              avg_landmark_guess = avg_landmark_guess + particles(pIdx).landmarks(lIdx).pos;
          end

          % Compute the average position by dividing by the number of particles
          avg_landmark_guess = avg_landmark_guess / length(particles);

          % Plot the average position of the current landmark
          plot3(avg_landmark_guess(1), avg_landmark_guess(2), avg_landmark_guess(3), 'ko', 'MarkerSize', 10);
      end
  end

  % Plot the particles
  particles_pos = [particles.position];
  plot3(particles_pos(1,:), particles_pos(2,:), particles_pos(3,:), 'r.');

  % Plot the real robot
  plot3(pos_history(1,:), pos_history(2,:), pos_history(3,:), 'r', "LineWidth",1.5);
  plot3(mean_trajectory(1,:), mean_trajectory(2,:), mean_trajectory(3,:), 'g-',"LineWidth",1, "LineStyle","-");
  plot3(real_position(1), real_position(2), real_position(3), 'mo', ...
                                           'LineWidth',1.5, ...
                                           'MarkerEdgeColor','k', ...
                                           'MarkerFaceColor',"#0072BD", ...
                                           'MarkerSize',10);

  % Show the sensor measurement as an arrow
  for lIdx=1:size(real_landmarks,2)
    real_landmark = real_landmarks(:, lIdx);
    if(read_distance(lIdx) < max_read_distance)
      line([real_position(1), real_position(1) + read_distance(lIdx) * cos(read_elevation(lIdx)) * cos(read_azimuth(lIdx))], ...
           [real_position(2), real_position(2) + read_distance(lIdx) * cos(read_elevation(lIdx)) * sin(read_azimuth(lIdx))], ...
           [real_position(3), real_position(3) + read_distance(lIdx) * sin(read_elevation(lIdx))],'LineStyle','--');
    end
  end

  axis([-5, 5, -5, 5, 0, 1.5]);
  grid minor;
  pause(.01);
end



