function [path, control, cost] = OCP7t(N, lhd_length_rear_from_axle, lhd_length_front_from_axle , ...
    start, goal, ...
    max_steering_radians, max_velocity, max_steering_velocity, ...
    plot_flag, movie_flag)
%% Optimimization-based two-point-value solver with 5WS kinematics ...
%       ... w/ max steering velocity and velocity constraints
%   vehicle configuration : (x1, y1, x2,y2, theta1, theta2, delta) where
%   1 refer to the rear part and 2 to the front part.
% Model taken from Altafini, C. (1999). A Path-Tracking Criterion for an LHD Articulated Vehicle. The International Journal of Robotics Research, 18, 435 - 441.
%%

 %initialize returned values
 cost = 0;
 path = 0;
 control = 0;

 opti = casadi.Opti();
  
 % ---- decision variables ---------
 X  =   opti.variable(7,N+1);
     x1     =       X(1,:);
     y1     =       X(2,:);
     th1    =       X(3,:);
     x2      =      X(4,:);
     y2      =      X(5,:);
     th2     =      X(6,:);
     delta  =       X(7,:);
 
 % ---- control input ---------       
 U  =   opti.variable(2,N);
     v        =       U(1,:);
     ddelta     =     U(2,:);
     
 T = opti.variable();       % final time
 
 % ---- objective          ---------
 opti.minimize(T);          % find the min-time path
 
 % --------------- dynamics --------------

 L1 = lhd_length_rear_from_axle;
 L2 = lhd_length_front_from_axle;
f = @(x,u) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    (u(1)*sin(x(7)) - u(2)*L2)/(L2 + L1*cos(x(7)));
    u(1)*cos(x(6));
    u(1)*sin(x(6));
    (u(1)*sin(x(7)) + L2*cos(x(7))*u(2))/(L2 + L1*cos(x(7)))
    u(2)];

 

 dt = T/N; % length of a control interval
 for k = 1 : N % loop over control intervals
    % Runge-Kutta 4 integration
    k1 = f(X(:,k),         U(:,k));
    k2 = f(X(:,k)+dt/2*k1, U(:,k));
    k3 = f(X(:,k)+dt/2*k2, U(:,k));
    k4 = f(X(:,k)+dt*k3,   U(:,k));
    x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
    opti.subject_to(X(:,k+1) == x_next); % close the gaps
%     if k > 1
%         opti.subject_to( (X(6,k)*sin(X(4,k) - X(5,k))/(distance_between_axes*cos(X(4,k))) + U(3,k))^2	<= (X(6,k)*max_tangential_velocity^2/max_centipedal_acceleration)^2);
%     end

 end
 
 % --- state constraints ---
 opti.subject_to( -max_steering_radians <= delta <= max_steering_radians);
 

 % --- control constraints ---
 opti.subject_to( -max_velocity	<= v    <= max_velocity);
 opti.subject_to( -max_steering_velocity   <= ddelta  <= max_steering_velocity);

  
 % ---  boundary start conditions ---
 opti.subject_to( x2(1) == start(1) );
 opti.subject_to( y2(1) == start(2) );
 opti.subject_to( th2(1)== start(3) );
 opti.subject_to( delta(1) == start(4) );

 th1_i = start(4)+ start(3);
 x1_i = start(1) - L1*cos(th1_i) - L2*cos(start(3));
 y1_i = start(2) - L1*sin(th1_i) - L2*sin(start(3));

 opti.subject_to( th1(1) == th1_i);
 opti.subject_to( x1(1) == x1_i);
 opti.subject_to( y1(1) == y1_i);
 

 % ---  boundary goal conditions ---
 opti.subject_to( x2(N+1) == goal(1) );
 opti.subject_to( y2(N+1) == goal(2) );
 opti.subject_to( th2(N+1)== goal(3) );
 opti.subject_to( delta(N+1) == goal(4));


 
 
 % ---- misc. constraints  ----------
 opti.subject_to(T>=0) % Time must be positive
 %opti.subject_to(T/N >= 0.1) % The path should be drivable (avoid infisible control rate)

 % ---- initial values for solver ---
 opti.set_initial(T, norm(start(1:2)-goal(1:2))*10/max_velocity);

 opti.set_initial(x2, linspace(start(1),goal(1),(N+1)));
 opti.set_initial(y2, linspace(start(2),goal(2),(N+1)));
 opti.set_initial(v,1)
 opti.set_initial(ddelta,1)
    
 try
    opti.solver('ipopt'); % set numerical backend
    sol = opti.solve();
    path = [sol.value(x2)',sol.value(y2)',sol.value(th2)',sol.value(delta)',sol.value(th1)',sol.value(x1)',sol.value(y1)'];
    path = round(path, 3);
    control = [sol.value(v)',sol.value(ddelta)'];
    control = round(control, 3);
    %check curvature
    
%    approxError = 1e-5;
%     for i = 2 : size(path,2)
%         dl = path(i,4);
%         ddl = control(i-1,3);
%         if abs(v) > approxError && abs(v*sin(dl-phir)/(distance_between_axes*cos(dl)) + ddl) > abs(v)*max_centipedal_acceleration/max_tangential_velocity^2
%             fprintf('WARNING: Max curvature is higher than the max allowed at path point i-th: %d.\n.',i);
%         end
%         if abs(v) > approxError && abs(v*sin(dl-phir)/(distance_between_axes*cos(dl)) + ddl) > abs(v)*max_centipedal_acceleration/1.5^2
%            fprintf('Ok with 1.5 m/s.\n');
%         end
%     end
    cost = sol.value(T);
 catch
   opti.debug;
   fprintf("NO SOLUTION?");
   return;
 end
 
 %plot the motion primitive
 if plot_flag
    hold on
    plot(path(:,1),path(:,2), '-o');
    grid on; axis equal;
 end
 
 %display the movie
 if movie_flag
     for i=1:N
            h = draw_LHD_2(path(i,1:5)',[lhd_length_front_from_axle,lhd_length_rear_from_axle]');
            drawnow
            pause(0.05)
            if i > 1 && i < N
                for k=1:length(h)
                    set(h{k},'Visible','off');
                end
            end
      end
 end
     
end

