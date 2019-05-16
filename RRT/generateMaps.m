function [ map_struct ] = generateMaps(mapFileName, numMaps, start_state, goal_state)
%
%
%
% Inputs:
%       mapFileName - string of the file name (including local path)
%       numMaps - number of maps to be generated
%
% Outputs:
%       generatedMap_cell - cell array containing numMaps+1 maps
%                           the first element contains the seed map
%
 
map_seed = im2double(rgb2gray(imread(mapFileName)));

bridge_ind = find(map_seed ~= 0 & map_seed ~= 1);   % locations in the map where a bridge exists
bridge_probs = map_seed(bridge_ind);                % probability for those bridges
N = length(bridge_ind);                            % Number of bridges

% Holds the output for the generated bridges
generatedMap_cell = cell(numMaps,1);                

for i = 1:numMaps
   temp_map = map_seed;
   rand_vals = rand(N,1);   % generate a random value for each bridge
   bridge_vals = rand_vals < bridge_probs;  % are the bridges open?
   temp_map(bridge_ind) = bridge_vals;  % create the maps
   generatedMap_cell{i} = temp_map; % store the maps in the cell array
   
   %plot the maps for testing
   figure(i);
   imshow(imresize(temp_map,5,'nearest'));
   
end

[N, M] = size(map_seed);
[x,y] = meshgrid(1:N,1:M); 

map_struct.map_name = mapFileName;
map_struct.bridge_locations = [x(bridge_ind)'; y(bridge_ind)'];
map_struct.bridge_probabilities = bridge_probs;
map_struct.seed_map = map_seed;
map_struct.map_samples = generatedMap_cell;
map_struct.start = start_state;
map_struct.goal = goal_state;

end

