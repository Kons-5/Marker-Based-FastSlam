function [z, H, p] = getMeasurement(pos, landmark_pos, observation_variance)
    %   Given a landmark position and a robot position, this method will return a
    %   "measurement" z that contains the distance and the angle to the landmark.
    %   Gaussian random noise is added to both based on the variances given in the
    %   diagonal of the observation_variance matrix. Note that this method is used
    %   both to take a "real" measurement in the simulation, as well as to assess 
    %   what kind of measurement each of our hypothetical particles would take.
    %   This method also computes the Jacobian of the measurent function for use
    %   in an extended Kalman filter.


    % Compute the distance from the current position to the landmark, and add
    % some Gaussian noise to make things interesting. Note that we are using
    % a smaller variance in this Gaussian distribution, as the algorithm seems
    % to work better when it underestimates the quality of the sensor. 
    vector_to_landmark = [landmark_pos(1) - pos(1); landmark_pos(2) - pos(2); landmark_pos(3) - pos(3)];
    landmark_distance = norm(vector_to_landmark);
    landmark_distance = landmark_distance + normrnd(0, observation_variance(1)*.25);
    
    % Compute the azimuthal angle from the given pos to the landmark
    landmark_azimuth = atan2(vector_to_landmark(2), vector_to_landmark(1)) - pos(4);
    landmark_azimuth = landmark_azimuth + normrnd(0, observation_variance(2)*.25);
    landmark_azimuth = mod(landmark_azimuth + pi, 2*pi) - pi; % normalize the angle to be within [-pi, pi]
    
    % Compute the elevation angle given pos to the landmark
    landmark_elevation = atan2(vector_to_landmark(3), sqrt(vector_to_landmark(1)^2 + vector_to_landmark(2)^2));
    landmark_elevation = landmark_elevation + normrnd(0, observation_variance(3)*.25);
    landmark_elevation = mod(landmark_elevation + pi/2, pi) - pi/2; % normalize the angle to be within [-pi/2, pi/2]
    
    % Compute the Jacobian of this measurement function
    q = landmark_distance^2.0;
    p = vector_to_landmark(1)^2.0 + vector_to_landmark(2)^2.0;

    H = [vector_to_landmark(1)/sqrt(q), vector_to_landmark(2)/sqrt(q), vector_to_landmark(3)/sqrt(q);
        -vector_to_landmark(2)/q, vector_to_landmark(1)/q, 0;
        vector_to_landmark(3) * vector_to_landmark(1)/(sqrt(p) * q), ... 
        vector_to_landmark(3) * vector_to_landmark(2)/(sqrt(p) * q), -sqrt(p)/q];
    
    z = [landmark_distance; 
       landmark_azimuth;
       landmark_elevation];

    z = [landmark_pos(1) - pos(1); landmark_pos(2) - pos(2); landmark_pos(3) - pos(3)];
    H = eye(3);
end
