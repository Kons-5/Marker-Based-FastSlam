function newpos = updateMovement(pos, movement, variance, timestep)
% Compute how the robot should move from "pos" given the requested movement and
% some Gaussian random noise using the velocity motion model. 

  % Add some Gaussian random noise to the movement. 
  Linear_vel = normrnd(movement(1), variance(1)*.25);
  Angular_vel = normrnd(movement(2), variance(2)*.25);

  delta = zeros(3,1);
  delta(1,1) = - (Linear_vel / Angular_vel) * sin(pos(3)) +  (Linear_vel / Angular_vel)* sin(pos(3) + Angular_vel * timestep);
  delta(2,1) = (Linear_vel / Angular_vel) * cos(pos(3)) -  (Linear_vel / Angular_vel)* cos(pos(3) + Angular_vel * timestep);
  delta(3,1) = Angular_vel * timestep;

  newpos = pos+delta;
end
