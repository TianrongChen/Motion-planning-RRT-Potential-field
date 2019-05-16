%       params - params used in the motion model
%               .length - car legnth [m]
%               .width - car width [m]
%               .wb - wheel base [m]
%               .l_radius - left wheel radius [m]
%               .r_radius - right wheel radius [m]
%               .d_theta_nom - nominal theta change for each wheel [rad] 
%               .d_theta_max_dev - max deviation from nominal [rad]
%               .d_theta_reverse - d_theta for both wheels going in reverse [rad]
%               .max_moveCount - state.moveCount gets this value in case of
%               collision

params.length = 3;
params.width = 2; 
params.border = [params.length/2  params.length/2 -params.length/2 -params.length/2;...
                 params.width/2  -params.width/2  -params.width/2   params.width/2;...
                 1                1                1                1             ];
params.wb = 2;
params.l_radius = .25;
params.r_radius = .25;
params.d_theta_nom = .6;
params.d_theta_max_dev = .2;
params.d_theta_reverse = params.d_theta_nom/3;
params.max_moveCount = 100000;
params.observation_radius = 5;