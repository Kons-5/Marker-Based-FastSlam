clear; clf; clc; close all

%% Set figure size
set(gcf,"OuterPosition", [100 100 600 600]);

%% PARAMETERS

% The number of timesteps for the simulation
timesteps = 1000;

% The maximum distance and angle from which our sensor can sense a landmark
max_read_distance = 3.0;            % maximum range of the camera
max_read_angle = 3.14159 * (1/3);   % PI x constant = ]0, 2PI]

% The actual positions of the landmarks (each column is a separate landmark)
real_landmarks = [-3.0, 2.0, 0.0, 0.0, 1.0, 0.0, -1.0, 1.5, 0.5;   % x
                   2.0, 2.5, 3.4, 1.5, 3.5,-3.0,  2.0, 0.0, 4.0;   % y
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0];  % z


% The initial starting position of the robot
real_position = [ 0.0;      % x
                 -2.0;      % y
                  0.0];  % rotation

% The movement command given to the robot at each timestep                 
movement_command = [.025;     % Linear Velocity
                    .01];    % Angular Velocity
                    
% The Gaussian variance of the movement commands
movement_variance = [.030;   % Linear Velocity
                     .025];  % Angular Velocity

M = [movement_variance(1), 0.0;
     0.0, movement_variance(2)];

% The Gaussian variance of our sensor readings
measurement_variance = [0.100;  % Distance
                        0.010]; % Angle
                        

R = [measurement_variance(1), 0.0;
     0.0, measurement_variance(2)];

% Create the particles and initialize them all to be in the same initial
% position. 
particles = [];
num_particles = 200;
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
  real_position = moveParticle(real_position, movement_command, [0;0], delta_t);
  pos_history = [pos_history, real_position];

  % Move the actual particles
  for pIdx = 1:num_particles
    particles(pIdx).position = moveParticle(particles(pIdx).position, movement_command, movement_variance, delta_t);
  end

  % Try to take a reading from each landmark
  doResample = false;
  for lIdx = 1:size(real_landmarks,2)
    real_landmark = real_landmarks(:, lIdx);

    % Take a real (noisy) measurement from the robot to the landmark
    [z_real, G] = getMeasurement(real_position, real_landmark, measurement_variance);
    read_distance(lIdx) = z_real(1);
    read_angle(lIdx)    = z_real(2);

    % If the landmark is close enough and within FOV, then we can spot it
    if(read_distance(lIdx) < max_read_distance && abs(read_angle(lIdx)) < max_read_angle/2)
      doResample = true;

      for pIdx = 1:num_particles
        if(particles(pIdx).landmarks(lIdx).seen == false)
          % If we have never seen this landmark, then we need to initialize it.
          % We'll just use whatever first reading we received.
          particles(pIdx).landmarks(lIdx).pos = [particles(pIdx).position(1) + cos(particles(pIdx).position(3)+read_angle(lIdx))*read_distance(lIdx);
                                                 particles(pIdx).position(2) + sin(particles(pIdx).position(3)+read_angle(lIdx))*read_distance(lIdx)];

          % Initialize the landmark position covariance
          particles(pIdx).landmarks(lIdx).E = inv(G) * R * inv(G)';

          particles(pIdx).landmarks(lIdx).seen = true;

        else
          % Get previous learned position
          [z_p, Gp] = getMeasurement(particles(pIdx).position, particles(pIdx).landmarks(lIdx).pos, [0;0]);
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
  particle_positions = reshape([particles.position], 3, num_particles); 
  mean_position = mean(particle_positions, 2);
  mean_trajectory = [mean_trajectory, mean_position];

  % Resample all particles based on their weights
  if(doResample)
    particles = resample(particles);
  end

  %% PLOTTING

  clf;
  hold on;
  
  % Plot the landmarks
  for lIdx=1:size(real_landmarks,2)
    plot(real_landmarks(1,lIdx), real_landmarks(2,lIdx), "Marker","pentagram", "MarkerSize",10,"Color","b", "MarkerFaceColor","b");
  end

  % Loop over each landmark
  for lIdx = 1:size(real_landmarks, 2)
      % Check if the landmark has been seen by at least the first particle (assuming similar for others)
      if particles(1).landmarks(lIdx).seen

          % Initialize the average landmark guess for the current landmark
          avg_landmark_guess = zeros(2, 1);  % Use zeros for initialization
          avg_covariance = zeros(2, 2);

          % Sum positions of this landmark from all particles
          for pIdx = 1:length(particles)
              avg_landmark_guess = avg_landmark_guess + particles(pIdx).landmarks(lIdx).pos;
              avg_covariance = avg_covariance + particles(pIdx).landmarks(lIdx).E(1:2, 1:2);
          end

          % Compute the average position by dividing by the number of particles
          avg_landmark_guess = avg_landmark_guess / length(particles);

          % Plot the average position of the current landmark
          %plot(avg_landmark_guess(1), avg_landmark_guess(2), 'ko', 'MarkerSize', 10);

          % Plot the covariance ellipse for this landmark
          plot_covariance_ellipse(avg_landmark_guess(1), avg_landmark_guess(2), avg_covariance);
      end
  end

  % Plot the particles
  particles_pos = [particles.position];
  plot(particles_pos(1,:), particles_pos(2,:), 'r.');

  % Plot the real robot
  plot(pos_history(1,:), pos_history(2,:), 'r', "LineWidth",1.5);
  plot(mean_trajectory(1,:), mean_trajectory(2,:), 'g-',"LineWidth", 1.5, "LineStyle","-");
  plot(real_position(1), real_position(2), 'mo', ...
                                           'LineWidth',1.5, ...
                                           'MarkerEdgeColor','k', ...
                                           'MarkerFaceColor',"#0072BD", ...
                                           'MarkerSize',10)
  % Plot the sensor cone (max FOV)
  draw_sensor_cone(real_position, max_read_distance, max_read_angle);

  % Show the sensor measurement as an arrow
  for lIdx=1:size(real_landmarks,2)
    real_landmark = real_landmarks(:, lIdx);
    if(read_distance(lIdx) < max_read_distance && abs(read_angle(lIdx)) < max_read_angle/2)
      line([real_position(1), real_position(1)+cos(real_position(3)+read_angle(lIdx))*read_distance(lIdx)], ...
           [real_position(2), real_position(2)+sin(real_position(3)+read_angle(lIdx))*read_distance(lIdx)], 'LineStyle','--');
    end
  end

  axis([-5, 5, -5, 5]);
  grid minor;

  xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 14)
  ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 14)

  pause(.01);
end

%% Auxiliary functions
function plot_covariance_ellipse(x, y, P)
    % x, y: Coordinates of the landmark
    % P: 2x2 Covariance matrix of the landmark position

    [V, D] = eig(P);  % Eigen decomposition of the covariance matrix
    % V: Eigenvectors; these form the axes of the ellipse
    % D: Eigenvalues; these define the length of each axis

    % Calculate the angle to rotate the ellipse
    theta = atan2(V(2,1), V(1,1));
    
    % Prepare to plot the ellipse
    steps = 0:0.01:(2*pi);
    ellipse_x = sqrt(D(1,1)) * cos(steps);
    ellipse_y = sqrt(D(2,2)) * sin(steps);

    % Rotation matrix
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    % Rotate and offset the ellipse to the correct position
    ellipse = R * [ellipse_x; ellipse_y];
    ellipse(1,:) = ellipse(1,:) + x;
    ellipse(2,:) = ellipse(2,:) + y;

    % Plotting the ellipse
    plot(ellipse(1,:), ellipse(2,:), 'r');

    % Plotting the "cross"
    hold on;
    % Major axis
    major_axis = V(:,1) * sqrt(D(1,1));
    plot([x - major_axis(1), x + major_axis(1)], [y - major_axis(2), y + major_axis(2)], 'r');
    % Minor axis
    minor_axis = V(:,2) * sqrt(D(2,2));
    plot([x - minor_axis(1), x + minor_axis(1)], [y - minor_axis(2), y + minor_axis(2)], 'r');
end

function draw_sensor_cone(position, max_distance, max_angle)
    % position: [x, y, theta] of the robot
    % max_distance: maximum distance the sensor can read
    % max_angle: maximum angle of the sensor field of view

    x = position(1);
    y = position(2);
    theta = position(3);

    % Define the angles of the cone edges
    angles = linspace(-max_angle/2, max_angle/2, 50);

    % Calculate the points of the cone boundary
    cone_x = [x, x + max_distance * cos(angles + theta)];
    cone_y = [y, y + max_distance * sin(angles + theta)];

    % Draw the cone as a filled polygon
    fill(cone_x, cone_y, 'y', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end