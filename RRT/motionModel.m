function [state_out, observed_map, flags] = motionModel(params, state_in, action, observed_map, actual_map, goal_state)
%MOTIONMODEL - Transitions the robot from one state to the next using a
%   differential drive motion model.
%
%
% Inputs:
%       params - params used in the motion model
%               .length - car legnth [m]
%               .width - car width [m]
%               .wb - wheel base [m]
%               .l_radius - left wheel radius [m]
%               .r_radius - right wheel radius [m]
%               .d_theta_nom - nominal theta change for each wheel [rad] 
%               .d_theta_max_dev - max deviation from nominal [rad]
%               .d_theta_reverse - d_theta for both wheels going in reverse [rad]
%               .max_moveCount - state.moveCount gets this value in case of collision
%               .observation_radius - radius for which the robot can
%                                       observe its environment
%
%       state_in - Input state of the robot 
%               .x - x coordinate
%               .y - y coordinate
%               .theta - angular heading of vehicle
%               .moveCount
%       action - [-2 or -1:1] - action to be taken 
%               -2 is reverse
%               [-1:1] specifies the deviation from nominal forward d_theta
%       observed_map - map of the world the robot has seen so far
%       actual_map - ground truth map
%       goal_state - The desired (x,y) position
%                   .x
%                   .y
%
% Outputs:
%       state_out - Output state given the action
%       observed_map - The map of the world the car has seen
%       flags - 
%               0: continue
%               1: reached goal
%               2: collision detected
%
% Refs:
%       For more information on the diff drive model being used below, see:
%       http://rossum.sourceforge.net/papers/DiffSteer/
%

%*****************************
% Determine New State
%*****************************
    
if (action == -2)    % handle reverse motion
    
    r_dTheta = -params.d_theta_reverse;
    l_dTheta = -params.d_theta_reverse;
    
    R = params.r_radius*r_dTheta;    % distance Right wheel traveled
    L = params.l_radius*l_dTheta;    % distance Left wheel traveled
        
    % Car moved straight backwards
    state_out.x = state_in.x + (R+L)/2*cos(state_in.theta);
    state_out.y = state_in.y + (R+L)/2*sin(state_in.theta);
   
    state_out.theta = state_in.theta + (R-L)/params.wb;
    state_out.moveCount = state_in.moveCount + 1;
    
    flags = 0; % continue flag
    
else
    
    % handle illegal actions
    if (action < -1 || action > 1)
        display('Illegal Action Requested');
        state_out = state_in;
        state_out.moveCount = state_out.moveCount + 1;
        flags = 0; % continue flag
        return;
    end
    
     
    r_dTheta = params.d_theta_nom + params.d_theta_max_dev*action;
    l_dTheta = params.d_theta_nom - params.d_theta_max_dev*action;
    
    R = params.r_radius*r_dTheta;    % distance Right wheel traveled
    L = params.l_radius*l_dTheta;    % distance Left wheel traveled
    
    if (R == L)
        % Car moved straight
        state_out.x = state_in.x + (R+L)/2*cos(state_in.theta);
        state_out.y = state_in.y + (R+L)/2*sin(state_in.theta);
    else
        % Car moved along an arc
        state_out.x = state_in.x + params.wb/2*(R+L)/(R-L)*(sin((R-L)/params.wb + state_in.theta) - sin(state_in.theta));
        state_out.y = state_in.y - params.wb/2*(R+L)/(R-L)*(cos((R-L)/params.wb + state_in.theta) - cos(state_in.theta));
    end

    state_out.theta = state_in.theta + (R-L)/params.wb;
    state_out.moveCount = state_in.moveCount + 1;
    
    flags = 0; % continue flag
end

%*****************************
% Transform initial car polygon into referse frame of the car
%*****************************
    
state_out.H = [cos(state_out.theta) -sin(state_out.theta) state_out.x;
               sin(state_out.theta)  cos(state_out.theta) state_out.y;
               1                     1                    1          ;];
state_out.border = state_out.H*params.border;


%*****************************
% detect collisions
%*****************************    

[N, M] = size(actual_map);

%get (x,y) locations on the map 
[x,y] = meshgrid(1:N,1:M); 

% find all 0 locations on the map
[ind] = find(actual_map==0);

% check to see whether any of these locations falls inside our car
% polygon
in = inpolygon(x(ind), y(ind), state_out.border(1,:), state_out.border(2,:));

if (sum(in)>0) 
    %we have a collision
    %display('Car Has Collided');        
    state_out = state_in;
    state_out.moveCount = params.max_moveCount;
    flags = 2; % collision flag
    return;
end

%*****************************
% update observed map
%*****************************

% get distances from each location to the car's location
dists = sqrt((x-state_out.x(ones(N,M))).^2 + (y-state_out.y(ones(N,M))).^2);

% find indices which are closer than observation radius
obs_ind = find(dists <= params.observation_radius);

% replace those indices in the observed map with values from the actual
% map
observed_map(obs_ind) = actual_map(obs_ind);


%*****************************
% Check goal
%*****************************

% check to see whether goal falls inside our car polygon
in = inpolygon(goal_state.x, goal_state.y, state_out.border(1,:), state_out.border(2,:));

if (sum(in)>0) 
    % we have reached the goal, time to bust out the beer!
    display('Reached the Goal!');        
    flags = 1; % goal flag
    return;
end


end

