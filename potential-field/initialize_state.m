% Initialize observed map
observed_map = map_struct.seed_map;
 
goal = map_struct.goal;

state.x = map_struct.start.x;
state.y = map_struct.start.y;
state.theta = 0; % theta always defaults to 0

state.H = [cos(state.theta) -sin(state.theta) state.x;
           sin(state.theta)  cos(state.theta) state.y;
           1                 1                1      ;];
state.border = state.H*params.border;

state.moveCount = 0;    

% reset flags
flags = 0;