% CarLikeVehicleMPrimGenerator : Generates a primitive set for a
% Car-like vehicle. The set guarantees 8-axis simmetry
%
%   output_filename :           the file to which the results are saved
%   numberofangles :            the number of orientation angles for this vehicle -- MUST BE MULTIPLE OF 8
%   steeringanglepartitions :   the number of steering angle partitions for this vehicle, calculated as (2*pi)/steeringAngleGranularity
%   steeringanglecardinality :  the number of steering angles supported by the vehicle -- MUST BE ODD AND <= steeringanglepartitions
%   max_steering_radians :      max steering of the vehicle
%   distance :                  the max distance for the area of the primitives (meters)
%   cell_size :                 the size of the side of each cell (meters)
%
%   all measures are in radians and meters
%   Two outputs: all primitives and primitive set
%
% Note:  no steering -> steeringanglepartitions == steeringanglecardinality == 1
addpath("~/matlab/casadi-linux-matlabR2014b-v3.5.5/")
addpath('plotting');
close all;
import casadi.*
%%%%%%%%%%%%%%%%%%%%%%%%

% -------------------------------------------------------------------------
% ========================== MACHINE PARAMETERS ============================
% -------------------------------------------------------------------------
lhd_width =                     2.490;
lhd_length_back =               4.955;
lhd_length_front =              3.615;
lhd_length_back_from_axle =     3.8;
lhd_length_front_from_axle =    1.2; 
distance_between_axes =         4.175;
max_steering_radians =          0.7;
max_velocity_forward =          20;
max_velocity_reverse =          8;
max_steering_velocity =         1;
max_acceleration =              1;
%%%%%%%%%%%%%%%%%%%%%%%%%

% -------------------------------------------------------------------------
% ========================== SCRIPT PARAMETERS ============================
% -------------------------------------------------------------------------
output_filename =               'volvo_l70';
numberofangles =                8;
steeringanglepartitions =       16;
steeringanglecardinality =      3;
distance =                      10;
cell_size =                     1.0;
N =                             100;
system_type =                   8; 
plotting =                      0;
plot_flag =                     0;
movie_flag =                    0;

% -------------------------------------------------------------------------
% ============================= PARAMETERS ================================
% -------------------------------------------------------------------------

% Cost multipliers
forwardcostmult = 1.0;
forwardturncostmult = 1.2; % 1.2;
crabcostmult = 1.0; %1.1;
parkingcostmult = 1.0; %2.0;
backwardcostmult = 1.1; %1.0;
backwardturncostmult = 1.3; % 1.2;

% approximation error tolerance
approxError = 10e-05;

% max hinge angle rate in rad/s
max_steering_angle_rate = 1;

% assumed longitudinal speed to calculate the hinge angle rate (m/s)
assumed_longitudinal_speed = 1.0;
%assumed_longitudinal_speed = 0.4;

% primitives connecting point A and B should not be longer than
% allowed_primitive_length_multiplier * linear_distance(A,B)
allowed_primitive_length_multiplier = 1.5; %%1.3
crab_distance_gain = 1.2; %how much this primitive should be cosidered good

% parameter to avoid primitives which curve too much on one side
max_side_angle = pi/4;

% parameter to avoid primitives which change too much the orientation of
% the vehicle
max_orient_change = (pi/2);

%
% -------------------------------------------------------------------------
% ======================== PRIMITIVE GENERATION ===========================
% -------------------------------------------------------------------------

tic;
base_primitives_filename = sprintf('%s_all_baseprims.mat', output_filename);

% orientation and steering granularity + the initial steering ID
steering_angle_granularity = 2*pi / steeringanglepartitions;
orientation_angle_granularity = pi/(numberofangles/2);
initial_steer_ID = mod((steeringanglepartitions - ((steeringanglecardinality - 1) / 2)),steeringanglepartitions) + 1;

% we have already generated all the base primitives
if exist(base_primitives_filename,'file') ~= 0
    load(base_primitives_filename);
else
    %matlabpool open 8
    pool = parpool(8);
    
    % generate the base primitives for this model, that is, the ones in the first sector  [0  pi/4]
    start_x = 0;
    start_y = 0;
    
    % motion primitive orientationID, motion primitive steeringID
    all_base_primitives = cell(numberofangles/8+1, steeringanglecardinality,1);
    all_base_primitives_motions = cell(numberofangles/8+1,steeringanglecardinality,1);
    all_base_primitives_costs = cell(numberofangles/8+1,steeringanglecardinality,1);
    for i = 1:size(all_base_primitives,1)
        for j = 1:size(all_base_primitives,2)
            all_base_primitives{i,j} = cell(1,0);
            all_base_primitives_motions{i,j} = cell(1,0);
            all_base_primitives_costs{i,j} = cell(1,0);
        end
    end

fp = figure;
title("Primitive")  

% counter of pre-discarded primitives
pre_discarded_counter = 0;   

    % orientation and steering
    for start_orient_ID = 1:numberofangles/8+1
        for start_steer_ID = 1:steeringanglecardinality
            
            real_start_steer_ID = initial_steer_ID + start_steer_ID - 1;
            if real_start_steer_ID > steeringanglepartitions
                real_start_steer_ID = real_start_steer_ID - steeringanglepartitions;
            end
            
            start_o = (start_orient_ID - 1) * orientation_angle_granularity;
            start_o = atan2(sin(start_o),cos(start_o));
            
            start_phi = (real_start_steer_ID - 1) * steering_angle_granularity;
            start_phi = atan2(sin(start_phi),cos(start_phi));
           
            
            fprintf('\nStart pose: %2.4f %2.4f %2.4f %2.4f\n', start_x,start_y,start_o,start_phi);
            
            % goal x and y
            %for goal_x =  -distance : cell_size : distance
                %for goal_y = -distance : cell_size : distance
            for goal_x =  -distance : cell_size : distance
                for goal_y = -distance : cell_size : distance
                    % we do not consider the origin
                    if ~(goal_x == 0 && goal_y == 0)
                        
                        temp_primitives = cell(numberofangles+1,1);
                        temp_primitives_motions = cell(numberofangles+1,1);
                        temp_primitives_costs = cell(numberofangles+1,1);
                        
                        for ind = 1:size(temp_primitives,1)
                            temp_primitives{ind} = cell(steeringanglecardinality,1);
                            temp_primitives_motions{ind} = cell(steeringanglecardinality,1);
                            temp_primitives_costs{ind} = cell(steeringanglecardinality,1);
                            
                            for ind2 = 1:size(temp_primitives{ind},1)
                                temp_primitives{ind}{ind2} = cell(1,0);
                                temp_primitives_motions{ind}{ind2} = cell(1,0);
                                temp_primitives_costs{ind}{ind2} = cell(1,0);
                            end
                        end
                        
                        % goal orientation and steering
                        parfor goal_orient_ID = 1:numberofangles %parfor
                        %for goal_orient_ID = 1:numberofangles %parfor
                            for goal_steer_ID = 1:steeringanglecardinality 
                                
                                real_goal_steer_ID = initial_steer_ID + goal_steer_ID - 1;
                                if real_goal_steer_ID > steeringanglepartitions
                                    real_goal_steer_ID = real_goal_steer_ID - steeringanglepartitions;
                                end
                                
                                goal_o = (goal_orient_ID - 1) * orientation_angle_granularity;
                                goal_o = atan2(sin(goal_o),cos(goal_o));
                                
                                goal_phi = (real_goal_steer_ID - 1) * steering_angle_granularity;
                                goal_phi = atan2(sin(goal_phi),cos(goal_phi));
                                
                                fprintf('Goal pose: %2.4f %2.4f %2.4f %2.4f\n', goal_x,goal_y,goal_o, goal_phi);
                               
                                % -------------------------------------------------
                                %     Mitko's functions: find a suitable path
                                % -------------------------------------------------
                                if goal_x > start_x
                                    motion = 1;
                                else
                                    motion =-1;
                                end
                                   
                                
                               switch system_type
                                   case 7

                                   [intermcells_m, control, primitivecost] = OCP7t(N,lhd_length_back_from_axle,lhd_length_front_from_axle, ...
                                                        [start_x, start_y, start_o, start_phi], ...
                                                        [goal_x, goal_y, goal_o,goal_phi], ...
                                                        max_steering_radians, max_velocity_forward, max_steering_velocity, ...
                                                        plot_flag, movie_flag);

                                   case 8

                                   [intermcells_m, control, primitivecost] = OCP8t(N,lhd_length_back_from_axle,lhd_length_front_from_axle, ...
                                                        [start_x, start_y, start_o, start_phi], ...
                                                        [goal_x, goal_y, goal_o,goal_phi], ...
                                                        max_steering_radians, max_velocity_forward,max_velocity_reverse, max_steering_velocity, ...
                                                        max_acceleration, ...
                                                        plot_flag, movie_flag);


                                   otherwise
                                       error('Unexpected solver type.Choose beetween a 4 or 5 variables system \n')

                                end
                               
                            
                                if size(intermcells_m, 1) > 1 % we have a result

                                    % normalize the angles
                                    intermcells_m(:,3) = atan2(sin(intermcells_m(:,3)), cos(intermcells_m(:,3)));
                                    intermcells_m(:,4) = atan2(sin(intermcells_m(:,4)), cos(intermcells_m(:,4)));
                                   
                                    % remove -0
                                    intermcells_m(logical(abs(intermcells_m) < approxError)) = 0;
                                    % we do not want -3.14 but only 3.14
                                    intermcells_m((logical(intermcells_m(:,3) - approxError < -pi)), 3) = pi;
                                    intermcells_m((logical(intermcells_m(:,4) - approxError < -pi)), 4) = pi;
                                    

                                    % check if the path respects the steering constraints
                                    if size(find(abs(intermcells_m(:,4)) > max_steering_radians),1) > 0
                                        fprintf(1,'FAILURE: valid path not found (steering radians constraints violated) [%2.4f %2.4f %2.4f %2.4f] [%2.4f %2.4f %2.4f %2.4f] \n', ...
                                            start_x, start_y, start_o, start_phi, goal_x, goal_y, goal_o, goal_phi)
                                    else
                                        temp_primitives{goal_orient_ID}{goal_steer_ID} = intermcells_m;
                                        temp_primitives_motions{goal_orient_ID}{goal_steer_ID} = motion;
                                        temp_primitives_costs{goal_orient_ID}{goal_steer_ID} = primitivecost;
                                        % we found a good path
                                        fprintf(1,'SUCCESS: valid path found [%2.4f %2.4f %2.4f %2.4f] [%2.4f %2.4f %2.4f %2.4f] \n', ...
                                            start_x, start_y, start_o, start_phi, goal_x, goal_y, goal_o, goal_phi)
                                    end
                                else
                                    fprintf(1,'FAILURE: valid path not found [%2.4f %2.4f %2.4f %2.4f] [%2.4f %2.4f %2.4f %2.4f] \n', ...
                                        start_x, start_y, start_o, start_phi, goal_x, goal_y, goal_o, goal_phi)
                                end
                            end % goal steering angle
                         end %end parfor
                        
                        % save the valid primitives
                        for ind_orient = 1:numberofangles+1
                            for ind_steer = 1:steeringanglecardinality
                                if size(temp_primitives{ind_orient}{ind_steer},1) > 1
                                    all_base_primitives{start_orient_ID,start_steer_ID}{size(all_base_primitives{start_orient_ID,start_steer_ID},2)+1} = temp_primitives{ind_orient}{ind_steer};
                                    all_base_primitives_motions{start_orient_ID,start_steer_ID}{size(all_base_primitives_motions{start_orient_ID,start_steer_ID},2)+1} = temp_primitives_motions{ind_orient}{ind_steer};
                                    all_base_primitives_costs{start_orient_ID,start_steer_ID}{size(all_base_primitives_costs{start_orient_ID,start_steer_ID},2)+1} = temp_primitives_costs{ind_orient}{ind_steer};
                                end
                            end
                        end
                        
                    end
                end
            end
        end
    end
    % save the results for future use
    save(base_primitives_filename, 'all_base_primitives', 'all_base_primitives_motions', 'all_base_primitives_costs');
    %matlabpool close
    delete(gcp('nocreate'))
end % end check if all base prims exist


% -------------------------------------------------------------------------
% ================ FILTERING & ADDING COST MULTIPLIERS ====================
% -------------------------------------------------------------------------

% filtering: eliminate unwanted primitives
primitives = cell(numberofangles+2,steeringanglecardinality,1);
primitives_costs = cell(numberofangles+2,steeringanglecardinality,1);
temp_primitives = cell(numberofangles+2,steeringanglecardinality,1);
temp_primitives_motions = cell(numberofangles+2,steeringanglecardinality,1);
temp_primitives_costs = cell(numberofangles+2,steeringanglecardinality,1);
primitives_cost_multipliers = cell(numberofangles+2,steeringanglecardinality,1);
for i = 1:size(primitives,1)
    for j = 1:size(primitives,2)
        primitives{i,j} = cell(1,0);
        primitives_costs{i,j} = cell(1,0);
        temp_primitives{i,j} = cell(1,0);
        temp_primitives_motions{i,j} = cell(1,0);
        temp_primitives_costs{i,j} = cell(1,0);
        primitives_cost_multipliers{i,j} = cell(1,0);
    end
end


for angleind = 1:numberofangles/8+1
    for steerind = 1:steeringanglecardinality
        
        %----------------------------------------------------------------------
        % first filtering: primitives outside the scope or too long
        %----------------------------------------------------------------------
        for primind = 1:size(all_base_primitives{angleind,steerind},2)
            
            fprintf(1,'First filtering: primitive #%d (%d) -- orient %d (%d) -- steer %d (%d) \n', ...
                primind, size(all_base_primitives{angleind,steerind},2), ...
                angleind, numberofangles/8+1, steerind, steeringanglecardinality);
            
            original = all_base_primitives{angleind,steerind}{primind};
            traj_x = original(:,1);
            traj_y = original(:,2);
            traj_o = original(:,3);
            traj_d = original(:,4);
          
            
            start_x = traj_x(1,1);
            goal_x = traj_x(size(traj_x,1),1);
            start_y = traj_y(1,1);
            goal_y = traj_y(size(traj_y,1),1);
            start_o = traj_o(1,1);
            goal_o = traj_o(size(traj_o,1),1);
            
            valid_primitive = true;
            primitive_length = 0;
            
            
           % remove primitives that curve too much on the sides
           if valid_primitive && ...
                    (pi - abs(abs(start_o-atan2(goal_y,goal_x))-pi)) + approxError >= max_side_angle && ...
                    (pi - abs(abs(start_o+pi-atan2(goal_y,goal_x))-pi)) + approxError >= max_side_angle
                valid_primitive = false;
            end
            
            
             for index = 2:size(traj_x,1)
                primitive_length = primitive_length + sqrt((traj_x(index,1) - (traj_x(index-1,1)))^2 + (traj_y(index,1) - (traj_y(index-1,1)))^2);
            end
            if primitive_length > sqrt((goal_x - start_x)^2 + (goal_y - start_y)^2) * allowed_primitive_length_multiplier
                disp("Too long motion primitive");
                valid_primitive = false;
            end
            
            if valid_primitive
                temp_primitives{angleind,steerind}{size(temp_primitives{angleind,steerind},2)+1} = original;
                temp_primitives_motions{angleind,steerind}{size(temp_primitives_motions{angleind,steerind},2)+1} = all_base_primitives_motions{angleind,steerind}{primind};
                temp_primitives_costs{angleind,steerind}{size(temp_primitives_costs{angleind,steerind},2)+1} = all_base_primitives_costs{angleind,steerind}{primind};
            end
        end
        
        fprintf(1,'\nPrimitives after first filtering: %d (%d) \n\n', size(temp_primitives{angleind,steerind},2), size(all_base_primitives{angleind,steerind},2));
        
        %----------------------------------------------------------------------
        % second filtering: remove when hinge angle rate is too high
        %----------------------------------------------------------------------
        
        for primind = 1:size(temp_primitives{angleind,steerind},2)
            
            fprintf(1,'Second filtering: primitive #%d (%d) -- orient %d (%d) -- steer %d (%d) \n', ...
                primind, size(temp_primitives{angleind,steerind},2), ...
                angleind, numberofangles/8+1, steerind, steeringanglecardinality);
            
            original = temp_primitives{angleind,steerind}{primind};
            original_cost = temp_primitives_costs{angleind,steerind}{primind};
            traj_x = original(:,1);
            traj_y = original(:,2);
            traj_o = original(:,3);
            traj_phi = original(:,4);
            
            start_x = traj_x(1,1);
            goal_x = traj_x(size(traj_x,1),1);
            start_y = traj_y(1,1);
            goal_y = traj_y(size(traj_y,1),1);
            start_o = traj_o(1,1);
            goal_o = traj_o(size(traj_o,1),1);
            start_phi = traj_phi(1,1);
            goal_phi = traj_phi(size(traj_phi,1),1);
            
            valid_primitive = true;
            
            % HERE WE ELIMINATE WITH THE HINGE ANGLE RATE
            % allow for a small percentage of outliers
            hinge_error_counter = 1; % ceil(size(traj_x,2) * 0.01);
            for index = 2:size(traj_x,1)
                partial_distance = sqrt((traj_x(index,1) - (traj_x(index-1,1)))^2 + (traj_y(index,1) - (traj_y(index-1,1)))^2);
                dt = partial_distance / assumed_longitudinal_speed;
                current_rate = (abs(atan2(sin(traj_phi(index,1) - traj_phi(index-1,1)), cos(traj_phi(index,1) - traj_phi(index-1,1))))) / dt;
                if current_rate > max_steering_angle_rate * 2
                    if (temp_primitives_motions{angleind,steerind}{primind} ~= 2 && ...
                        temp_primitives_motions{angleind,steerind}{primind} ~= 3)
                        valid_primitive = false;
                    end
                else
                    if current_rate > max_steering_angle_rate
                        hinge_error_counter = hinge_error_counter - 1;
                    end
                    if hinge_error_counter <= 0          
                        if (temp_primitives_motions{angleind,steerind}{primind} ~= 2 && ...
                            temp_primitives_motions{angleind,steerind}{primind} ~= 3)
                            valid_primitive = false;
                        end
                    end
                end
                if valid_primitive == false
                    break
                end
            end
            
            if valid_primitive
                primitives{angleind,steerind}{size(primitives{angleind,steerind},2)+1} = original;
                primitives_costs{angleind,steerind}{size(primitives_costs{angleind,steerind},2)+1} = original_cost;
                
                % ---------------------------------------------------------
                % add cost multipliers
                % ---------------------------------------------------------
                goal_x = original(size(original, 1), 1);
                goal_y = original(size(original, 1), 2);
                goal_o = original(size(original, 1), 3);
                start_o = original(1, 3);
                
                % check if the motion is forward or backward and if we turn
                if temp_primitives_motions{angleind,steerind}{primind} == 1
                    % forward motion
                    if (pi - abs(abs(start_o - atan2(goal_y,goal_x))-pi)) < pi/32 && (pi - abs(abs(start_o - goal_o)-pi)) < pi/32 %%???
                        % straight
                        primitives_cost_multipliers{angleind,steerind}{size(primitives_cost_multipliers{angleind,steerind},2)+1} = forwardcostmult;
                    else
                        primitives_cost_multipliers{angleind,steerind}{size(primitives_cost_multipliers{angleind,steerind},2)+1} = forwardturncostmult;
                    end
                elseif temp_primitives_motions{angleind,steerind}{primind} == -1   
                    % backward motion
                    if (pi - abs(abs((start_o - pi) - atan2(goal_y,goal_x))-pi)) < pi/32 && (pi - abs(abs(start_o - goal_o)-pi)) < pi/32
                        % straight
                        primitives_cost_multipliers{angleind,steerind}{size(primitives_cost_multipliers{angleind,steerind},2)+1} = backwardcostmult;
                    else
                        primitives_cost_multipliers{angleind,steerind}{size(primitives_cost_multipliers{angleind,steerind},2)+1} = backwardturncostmult;
                    end    
                elseif temp_primitives_motions{angleind,steerind}{primind} == 2
                     primitives_cost_multipliers{angleind,steerind}{size(primitives_cost_multipliers{angleind,steerind},2)+1} = crabcostmult;
                     disp("Crab cost multi")
                elseif temp_primitives_motions{angleind,steerind}{primind} == 3
                     primitives_cost_multipliers{angleind,steerind}{size(primitives_cost_multipliers{angleind,steerind},2)+1} = parkingcostmult;
                     disp("Park cost multi")
                end
                
            else % not a valid primitive
                fprintf(1,'........ primitive #%d discarded \n', primind);
            end
        end
    end
end


% -------------------------------------------------------------------------
% ============================== SYMMETRIES ===============================
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% Second sector  (pi/4 - pi/2]
% -------------------------------------------------------------------------
% iterate over angles
for angleind = ((numberofangles/8)+2):numberofangles/4+1
    for steerind = 1:steeringanglecardinality
        % iterate over primitives
        for primind = 1:size(primitives{numberofangles/4 + 2 - angleind, steeringanglecardinality - steerind + 1},2)
            
            original = primitives{numberofangles/4 + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
            original_cost = primitives_costs{numberofangles/4 + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
            
            % invert x,y
            primitives{angleind,steerind}{primind}(:,1) = original(:,2);
            primitives{angleind,steerind}{primind}(:,2) = original(:,1);
            % invert the angles
            primitives{angleind,steerind}{primind}(:,3) = atan2(cos(original(:,3)), sin(original(:,3)));
            primitives{angleind,steerind}{primind}(:,4) = -original(:,4);
           
            
            % remove -0
            primitives{angleind,steerind}{primind}(logical(abs(primitives{angleind,steerind}{primind}) < approxError)) = 0;
            % we do not want -3.14 but only 3.14
            primitives{angleind,steerind}{primind}((logical(primitives{angleind,steerind}{primind}(:,3) - approxError < -pi)), 3) = pi;
            primitives{angleind,steerind}{primind}((logical(primitives{angleind,steerind}{primind}(:,4) - approxError < -pi)), 4) = pi;
            
            
            % add the cost multipliers
            primitives_cost_multipliers{angleind,steerind}{primind} = primitives_cost_multipliers{numberofangles/4 + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
            
            % add the cost
            primitives_costs{angleind,steerind}{primind} = original_cost;
        end
    end
end

% -------------------------------------------------------------------------
% 90 degrees rotation  (pi/2 - pi]
% -------------------------------------------------------------------------
% iterate over angles
for angleind = ((numberofangles/4)+2):numberofangles/2+1
    for steerind = 1:steeringanglecardinality
        % iterate over primitives
        for primind = 1:size(primitives{numberofangles/2 + 2 - angleind, steeringanglecardinality - steerind + 1},2)
            
            original = primitives{numberofangles/2 + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
            original_cost = primitives_costs{numberofangles/2 + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
            
            primitives{angleind,steerind}{primind}(:,1) = -original(:,1);
            primitives{angleind,steerind}{primind}(:,2) = original(:,2);
            % invert the angles
            primitives{angleind,steerind}{primind}(:,3) = atan2(sin(original(:,3)), -cos(original(:,3)));
            primitives{angleind,steerind}{primind}(:,4) = -original(:,4);
            %primitives{angleind,steerind}{primind}(:,5) = -original(:,5);
            
            % remove -0
            primitives{angleind,steerind}{primind}(logical(abs(primitives{angleind,steerind}{primind}) < approxError)) = 0;
            % we do not want -3.14 but only 3.14
            primitives{angleind,steerind}{primind}(logical(primitives{angleind,steerind}{primind}(:,3) - approxError < -pi), 3) = pi;
            primitives{angleind,steerind}{primind}(logical(primitives{angleind,steerind}{primind}(:,4) - approxError < -pi), 4) = pi;
     
            
            % add the cost multipliers
            primitives_cost_multipliers{angleind,steerind}{primind} = primitives_cost_multipliers{numberofangles/2 + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};

            % add the cost
            primitives_costs{angleind,steerind}{primind} = original_cost;
        end
    end
end

% -------------------------------------------------------------------------
% final rotation (pi - 2*pi)
% -------------------------------------------------------------------------
% iterate over angles
for angleind = numberofangles/2+2:numberofangles
    for steerind = 1:steeringanglecardinality
        % iterate over primitives
        for primind = 1:size(primitives{numberofangles + 2 - angleind, steeringanglecardinality - steerind + 1},2)
            
            original = primitives{numberofangles + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
            original_cost = primitives_costs{numberofangles + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
            
            primitives{angleind,steerind}{primind}(:,1) = original(:,1);
            primitives{angleind,steerind}{primind}(:,2) = -original(:,2);
            % invert the angles
            primitives{angleind,steerind}{primind}(:,3) = atan2(-sin(original(:,3)), cos(original(:,3)));
            primitives{angleind,steerind}{primind}(:,4) = -original(:,4);
   
            
            % remove -0
            primitives{angleind,steerind}{primind}(logical(abs(primitives{angleind,steerind}{primind}) < approxError)) = 0;
            % we do not want -3.14 but only 3.14
            primitives{angleind,steerind}{primind}(logical(primitives{angleind,steerind}{primind}(:,3) - approxError < -pi), 3) = pi;
            primitives{angleind,steerind}{primind}(logical(primitives{angleind,steerind}{primind}(:,4) - approxError < -pi), 4) = pi;
            
            % add the cost multipliers
            primitives_cost_multipliers{angleind,steerind}{primind} = primitives_cost_multipliers{numberofangles + 2 - angleind, steeringanglecardinality - steerind + 1}{primind};
            
            % add the cost
            primitives_costs{angleind,steerind}{primind} = original_cost;
        end
    end
end

toc

%%
% -------------------------------------------------------------------------
% ======================== OUTPUT AND PLOTTING ============================
% -------------------------------------------------------------------------

fout = fopen(output_filename, 'w');

% write the header
fprintf(fout, 'lhd_width: %f\n', lhd_width);
fprintf(fout, 'lhd_length_front: %f\n', lhd_length_front);
fprintf(fout, 'lhd_length_back: %f\n', lhd_length_back);
fprintf(fout, 'lhd_length_front_from_axle: %f\n', lhd_length_front_from_axle);
fprintf(fout, 'lhd_length_back_from_axle: %f\n', lhd_length_back_from_axle);
fprintf(fout, 'max_steering_radians: %f\n', max_steering_radians);
fprintf(fout, 'bound_phi: %f\n', max_steering_radians);
fprintf(fout, 'distance_between_axes: %f\n', distance_between_axes);
fprintf(fout, 'resolution_m: %f\n', cell_size);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'steeringanglepartitions: %d\n', steeringanglepartitions);
fprintf(fout, 'steeringanglecardinality: %d\n', steeringanglecardinality);

f1 = figure;
f2 = figure;

for angleind = 1 : numberofangles
    for steerind = 1:steeringanglecardinality
        % do we have primitives?
        for primind = 1:size(primitives{angleind,steerind},2)
            
            real_start_steer_ID = initial_steer_ID + steerind - 1;
            if real_start_steer_ID > steeringanglepartitions
                real_start_steer_ID = real_start_steer_ID - steeringanglepartitions;
            end
            
            fprintf(fout, 'primID: %d\n', primind-1);
            fprintf(fout, 'startangle_c: %d\n', angleind-1);
            fprintf(fout, 'startsteer_c: %d\n', real_start_steer_ID-1);
            
            intermcells_m = primitives{angleind,steerind}{primind};
            
            % add the cost multipliers, the cost of the motion primitive and motion directions
            fprintf(fout, 'additionalactioncostmult: %1.1f\n', primitives_cost_multipliers{angleind,steerind}{primind});
            if primitives_costs{angleind,steerind}{primind} > 0 %otherwise default
                fprintf(fout, 'primitivecost: %d\n', primitives_costs{angleind,steerind}{primind});
            end
            fprintf(fout, 'motiondirection: %d\n', 1);
            % write out
            fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
            
            % print to file
            for interind = 1:size(intermcells_m, 1)
                fprintf(fout, '%.4f %.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), intermcells_m(interind,3), intermcells_m(interind,4));
            end
            
            % -----------------------------------------------------------------
            % Plotting
            % -----------------------------------------------------------------
         
            if plotting == 1
                figure(f1);
                % THETA -- orientation
                subplot(2,2,3);
                plot(intermcells_m(:,3),'b')
                axis([1 size(intermcells_m,1) -pi pi]);
                grid on;
                title('theta')
                
                % PHI -- steering angle
                subplot(2,2,4);cla
                plot(intermcells_m(:,4),'b')
                axis([1 size(intermcells_m,1) -pi pi]);
                hold on;
                plot(linspace(max_steering_radians,max_steering_radians,size(intermcells_m,1)), 'r');
                hold on;
                plot(linspace(-max_steering_radians,-max_steering_radians,size(intermcells_m,1)), 'r');
                grid on;
                title('delta')
                
                % PATH with arrows
                subplot(2,1,1);cla
                hold on;
                text(0, 0, int2str(angleind));
                hold on;
                plot(intermcells_m(:,1), intermcells_m(:,2), '-');
                axis_limit = distance + 2*cell_size;
                axis([-axis_limit axis_limit -axis_limit axis_limit]);
                hold on;
                d = [cos(intermcells_m(1,3));sin(intermcells_m(1,3))]; d=d/norm(d)*1;
                draw_arrow([intermcells_m(1,1) intermcells_m(1,2)]', [intermcells_m(1,1) intermcells_m(1,2)]'+d, 0.1, 0.1, [0;0;0], [0;0;0]);
                hold on;
                d = [cos(intermcells_m(size(intermcells_m, 1),3));sin((intermcells_m(size(intermcells_m, 1),3)))]; d=d/norm(d)*0.5;
                draw_arrow([intermcells_m(size(intermcells_m, 1),1) intermcells_m(size(intermcells_m, 1),2)], [intermcells_m(size(intermcells_m, 1),1) intermcells_m(size(intermcells_m, 1),2)]'+d, 0.1, 0.1, [0;0;0], [0;0;0]);
                
                
                for i=1:5:size(intermcells_m, 1)
                    hold on;
                    d = [cos(intermcells_m(i,4)+intermcells_m(i,3));sin(intermcells_m(i,4)+intermcells_m(i,3))]; d=d/norm(d)*1;
                    draw_arrow([intermcells_m(i,1) intermcells_m(i,2)]', [intermcells_m(i,1) intermcells_m(i,2)]'+d, 0.01, 0.01, [0;0;0], [0;0;0]);
                end
                
                grid on;
                
                % ALL PATHS together
                figure(f2);
                hold on;
                plot(intermcells_m(:,1), intermcells_m(:,2), '-');
                axis([-axis_limit axis_limit -axis_limit axis_limit]);
                hold on;
                grid on;
                %pause;
                %clf
            end
        end
        
        size(primitives{angleind,steerind},2)
        if plotting == 1
            %pause;   %%%
            %clf
        end
    end
    
    %%% EOF
end

fclose('all');
