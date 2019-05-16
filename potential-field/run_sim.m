%% -----------------------------------------------------------------------

%*****************************
% Load map files and parameters
%*****************************

close all;
clear all;
clc;

%load map_1.mat;
%load map_2.mat;
load map_3.mat;

load_sim_params;
load_rrts_params;

% scale is used to blow up the environment for display purposes only. Set
% to whatever looks good on your screen
scale = 10;

% determines the size of the map and creates a meshgrid for display
% purposes
[N,M] = size(map_struct.seed_map);
[x,y] = meshgrid(1:N, 1:M);

DISPLAY_ON = 1; % 1 - turns display on, 0 - turns display off
DISPLAY_TYPE = 1; % 0 - displays map as dots, 1 - displays map as blocks

%% -----------------------------------------------------------------------

%*****************************
% Training/Learning Phase
%*****************************
%
% Here is where you perform any training or learning that your algorithm
% will require, such as solving an MDP, determining a policy or a value
% function, etc.
%
% NOTE: At this stage, you may access any variables provided in params or 
% map_struct EXCEPT:
%
%           map_struct.map_samples 
%
% This is your test data, don't touch them! You may however create your own 
% map_samples if you feel that this will be beneficial. You may use any of
% the functions provided (such as motion model) or create your own. Also
% note that your algorithm must be kept constant for any and all maps. You 
% cannot hand tweak parameters or settings in your solver for specific 
% maps.
%
% This code block is allowed to run as long as you'd like, provided that it
% finishes in time for you to submit the assignment!
%
% Example:
%
% myPolicy_struct = solve_mdp(params, map_struct, ...)
% ...

%[Q,V,road,wall,road_ind_map] = get_map(map_struct);

%% -----------------------------------------------------------------------

%*****************************
% Run Sim
%*****************************
%
% Here is where you'll actually run the simulation over the set of sampled
% maps. At each iteration, you will decide which action to take based on
% the state of the car, the observed map, and any training/learning data 
% that you may have created. There is no time limit on run time however you 
% must be able to run your solution on all map samples in time to submit 
% the assignment.

%u=[-2 linspace(-1,1,21)];
%data=zeros(1,params.max_moveCount);
%fan=zeros(3,22);
% Loop through each map sample
%%
for i = 39:length(map_struct.map_samples) 
%i
%pause
    % Initialize the starting car state and observed map
    % observed_map is set to seed map, and the bridge information will be
    % updated once the car is within params.observation_radius
    initialize_state;
   
    % display the initial state
    if (DISPLAY_ON)
        display_environment;
    end
    counter = 0;
    
    %my_map_struct=map_struct;
    old_map=zeros(size(observed_map));
    %control_count=1;
    % loop until maxCount has been reached or goal is found
    while (state.moveCount < params.max_moveCount && flags ~= 2)
    counter = counter+1;
        %---------------------------------------
        %
        %*****************************
        % Decide Action
        %*****************************
        %
        % Here you execute your policy (which may include re-planning or
        % any technique you consider necessary):
        
        
        % My example policy: slight turn
        %{
        if mod(counter,4)==1
            action=1;
        else
            action=-2;
        end
        %}
        %action = -1+(i-1)*2/39;
        %{
        if sum(sum(abs(sign(observed_map)-old_map))) ||control_count>length(result.control)
            result = myPlanPathRRTstar(params,rrts_param, state,observed_map, goal);
            old_map=sign(observed_map);
            control_count=1;
        end
        %}
        
        %%{
        if sum(sum(abs(sign(observed_map)-old_map))) ||control_count>length(my_control)
            %[my_control,Q,V,road,wall,u,v] = get_map(params,map_struct,observed_map,state);
            [my_control,Q,V,road,wall,u,v] = get_fine_map(params,map_struct,observed_map,state);
            old_map=sign(observed_map);
            control_count=1;
        end
        %}
        
        
        action=my_control(control_count);
        control_count=control_count+1;
        % Notice how with this policy, when the car gets close to the
        % unknown bridge (in map_1), on the first map sample the bridge 
        % becomes solid black (closed), and on the second map sample the 
        % bridge becomes white (open). Until the bridge is either 1 (white)
        % or 0 (black), you should treat it as unknown. 
        %
        % For display purposes, the unknown bridge is shown as a gray shade
        % proportional to its probability of being closed.
        %
        %---------------------------------------
        
        
        % Execute the action and update observed_map
        [state, observed_map, flags] = motionModel(params, state, action, observed_map, map_struct.map_samples{i}, goal);

        if flags == 1
            break;
        end
        if (DISPLAY_ON)&&(~mod(counter,40))
            display_environment;
        end

        % display some output
        
        %data(counter)=state.theta;
        %fan(:,i)=[state.x;state.y;state.theta];
        %fan(:,41)=[state.x;state.y];
      
        
        % pause if you'd like to pause at each step
        % pause;
        
    end
    %pause
end
