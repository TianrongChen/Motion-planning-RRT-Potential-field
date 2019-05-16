function result = myPlanPathRRTstar(params,rrts_param, cur_state,observed_map, p_goal)

% RRT* 
% credit : Anytime Motion Planning using the RRT*, S. Karaman, et. al.
% calculates the path using RRT* algorithm 
% param : parameters for the problem 
%   1) threshold : stopping criteria (distance between goal and current
%   node)
%   2) maxNodes : maximum nodes for rrt tree 
%   3) neighborhood : distance limit used in finding neighbors
%   4) obstacle : must be rectangle-shaped #limitation
%   5) step_size : the maximum distance that a robot can move at a time
%   (must be equal to neighborhood size) #limitation
%   6) random_seed : to control the random number generation
% p_start : [x;y] coordinates
% p_goal : [x;y] coordinates
% variable naming : when it comes to describe node, if the name is with
% 'node', it means the coordinates of that node or it is just an index of
% rrt tree
% rrt struct : 1) p : coordinate, 2) iPrev : parent index, 3) cost :
% distance
% obstacle can only be detected at the end points but not along the line
% between the points
% for cost, Euclidean distance is considered.
% output : cost, rrt, time_taken 
% whether goal is reached or not, depends on the minimum distance between
% any node and goal 

field1 = 'state';
field2 = 'iParent';
field3 = 'parent_control';
field4 = 'ichild';
field5 = 'moves';
field6 = 'goalReached';

rng(rrts_param.random_seed);
tic;
start();

    function start()      
        rrt(1) = struct(field1, cur_state, field2,[] , field3, [], field4, [],field5,0,field6,0);  
        N = rrts_param.maxNodes; % Nodes
        M = rrts_param.maxiter; % iterations
        j = 1;
        %load('rrt.mat')
        [map_x,map_y]=size(observed_map);
        %get (x,y) locations on the map 
        %[x,y] = meshgrid(1:map_x,1:map_y); 
        % find all 0 locations on the map
        %[ind] = find(observed_map==0);

%         while endcondition>param.threshold %&& j<=N 
        %figure
        %hold on
        for ii=1:M
            sample_node = getSample(map_x,map_y,observed_map);
             %plot(sample_node(1), sample_node(2), 'xk');
%             text(sample_node(1), sample_node(2), strcat('random',num2str(j)))
            nearest_node_ind = findNearest(rrt, sample_node);
             %plot(rrt(nearest_node_ind).state.x, rrt(nearest_node_ind).state.y, 'or');
%             text(rrt(nearest_node_ind).p(1), rrt(nearest_node_ind).p(2), strcat('nearest', num2str(j)));
            [new_state, parent_control, flags] = steering(rrt(nearest_node_ind).state, sample_node,rrt(nearest_node_ind).goalReached);
            %plot(new_state.x, new_state.y, '^b');
            
            if (flags~=2)% no collision
%                 plot(new_node(1), new_node(2), '.g');
%                 text(new_node(1), new_node(2)+3, strcat('steered: new node', num2str(j)))
                if flags
                    neighbors_ind=[];
                else
                    neighbors_ind = [];%getNeighbors(rrt, new_state);
                end
                if(~isempty(neighbors_ind))
                    [parent_node_ind,parent_control,new_state,flags] = chooseParent(rrt, neighbors_ind, nearest_node_ind,parent_control,new_state,flags);
                     %plot(new_state.x, new_state.y, '+k');
%                     text(rrt(parent_node_ind).p(1), rrt(parent_node_ind).p(2)+3, strcat('parent', num2str(j)));
                else
                    parent_node_ind = nearest_node_ind;
                end
                rrt = insertNode(rrt, parent_node_ind, new_state,parent_control,flags);
                j=j+1;
                %{
                if flags==1
                    break;
                end
                %}
                new_neighbors_ind = [];%getNeighbors(rrt, new_state);
                if (~isempty(new_neighbors_ind))
                    rrt = reWire(rrt, new_neighbors_ind, parent_node_ind, length(rrt));
                end
                
            end
            if ~mod(ii,50)
                [ii,j]
            end
            
            if j>N
                break
            end
        end
        figure(2)
        hold on;
        setPath(rrt);
%         text1 = strcat('Total number of generated nodes:', num2str(j-1))
%         text1 = strcat('Total number of nodes in tree:', length(rrt))
    end


    function setPath(rrt)
        plotx=zeros(1,length(rrt));
        ploty=plotx;
        for i = 1: length(rrt)
            plotx(i)=rrt(i).state.x;
            ploty(i)=rrt(i).state.y;
            %{
            p1 = [rrt(i).state.x rrt(i).state.y];
            rob.x = p1(1); rob.y=p1(2);
            figure(2)
            plot(rob.x,rob.y,'.b')
            
            for j = 1: length(rrt(i).ichild)
                p2 = [rrt(rrt(i).ichild(j)).state.x rrt(rrt(i).ichild(j)).state.y];
                figure(2)
                plot([p1(1),p2(1)], [p1(2),p2(2)], 'b', 'LineWidth', 1);
            end
            %}
        end 
        figure(2)
        plot(plotx,ploty,'.b')
        
        [cost,i] = getFinalResult(rrt);
        result.cost = cost;
        result.rrt = rrt;
        final_control=[];
        p11=[];
        while i ~= 0
            p11 = [p11 [rrt(i).state.x ;rrt(i).state.y]];
            final_control=[rrt(i).parent_control final_control];
            i = rrt(i).iParent;
        end  
        result.control=final_control;
        result.time_taken = toc;
        figure(2)
        hold on
        plot(p11(1,:),p11(2,:),'b', 'Marker','.', 'MarkerSize', 30);
        
    end

    function [value,min_node_ind] = getFinalResult(rrt)
        goal_ind = find([rrt.goalReached]==1);
        if ~(isempty(goal_ind))
            disp('Goal has been reached!');
            rrt_goal = rrt(goal_ind);
            [value,min_node_ind] = min(rrt_goal.moves);
            min_node_ind=goal_ind(min_node_ind);
            %u=
        else
            disp('Goal has not been reached!');
            norm_rrt=zeros(1,length(rrt));
            for i =1:length(rrt)
                norm_rrt(i) = norm([p_goal.x p_goal.y]-[rrt(i).state.x rrt(i).state.y]);
            end
            [~,min_node_ind]= min(norm_rrt); 
            value = rrt(min_node_ind).moves;
        end
    end
    
    function [new_state, u,flags]=steering(nearest_state, random_node,goal_flag)
        dp=random_node-[nearest_state.x, nearest_state.y];
        go=round(min(floor(norm(dp)/.15),1*rrts_param.neighbourhood/.15));
        %go=round(norm(dp)/.15);
        theta_new=atan2(dp(2),dp(1));
        dtheta=theta_new-nearest_state.theta;
        
        left_right=sign(dtheta);
        angle=abs(dtheta);
        %{
        if angle>pi
            angle=pi*2-angle;
            left_right=-left_right;
        end
        %}
        turn=floor(angle/rrts_param.maxturn);%.05
        res=angle-turn*rrts_param.maxturn;
        %%{
        u=[-2*ones(1,4*turn),left_right*res/rrts_param.maxturn,zeros(1,go)];
        ind=1:(4*turn);
        u(mod(ind,4)==3)=left_right;
        %}
        %{
        if ~turn
            u=[left_right*res/rrts_param.maxturn 0];
        elseif turn<32
            u=[left_right 0];
        else
            u=[-2 -2];
        end
        %}
        
        %run this u
        [new_state, ~, flags,ind] = run_actions(u,params, nearest_state, observed_map, p_goal,goal_flag);
        if flags==2 && ind>40
            u=u(1:ind);
            [new_state, ~, flags] = run_actions(u,params, nearest_state, observed_map, p_goal,goal_flag);
        end
    end

    function [state, observed_map, flags,ind] = run_actions(u,params, state, observed_map, goal,flags)
        ind=length(u);
        for i = 1:ind
            if (state.moveCount < params.max_moveCount && flags ~= 2)
                action=u(i);

                [state, observed_map, flags] = motionModel(params, state, action, observed_map, observed_map, goal);
                %DISPLAY_TYPE=1;
                %display_environment
                %figure(2)
                %scatter(state.x,state.y)
                %hold on

                if flags == 1
                    break;
                elseif flags==2
                    ind=i-6;
                    break
                end
            end
        end
    end
    
    function rrt = reWire(rrt, neighbors, parent, new)
        for i=1:length(neighbors)
            if isempty(rrt(neighbors(i)).ichild) && neighbors(i)~=parent
                [new_state, u,flags]=steering(rrt(new).state, [rrt(neighbors(i)).state.x rrt(neighbors(i)).state.y],rrt(new).goalReached);
            
                if (new_state.moveCount<rrt(neighbors(i)).moves)&& flags~=2
                    rrt(neighbors(i)).state = new_state;
                    rrt(neighbors(i)).iParent = new;
                    rrt(neighbors(i)).parent_control = u;
                    rrt(neighbors(i)).moves = new_state.moveCount;
                    rrt(neighbors(i)).goalReached = flags;
                end
            end
        end
    end
    

    function rrt = insertNode(rrt, parent, new_node,parent_control,flag)
        rrt(end+1) = struct(field1, new_node, field2, parent, field3, parent_control, field4, [],field5, new_node.moveCount,field6,flag);
        rrt(parent).ichild=[rrt(parent).ichild length(rrt)];
    end
    
    function [parent,parent_control,new_state_candidate,new_flag]= chooseParent(rrt, neighbors, nearest,parent_control, new_state,new_flags)
        min_cost = new_state.moveCount;%getCostFromRoot(rrt, nearest, new_state);
        parent = nearest;
        new_state_candidate=new_state;
        new_flag=new_flags;
        
        
        for i=1:length(neighbors)
            [new_neighbor_state,u, flags]=steering(rrt(neighbors(i)).state, [new_state.x new_state.y],rrt(neighbors(i)).goalReached);
            if (flags~=2)
                cost=new_neighbor_state.moveCount;
                if (cost<min_cost)
                   min_cost = cost;
                   parent = neighbors(i);
                   new_state_candidate=new_neighbor_state;
                   parent_control=u;
                   new_flag=flags;
                end
            end
        end
    end

    function neighbors = getNeighbors(rrt, node)
        neighbors = [];
        for i = 1:length(rrt)
            dist = norm([rrt(i).state.x-node.x, rrt(i).state.y-node.y]);
            if (dist<=rrts_param.neighbourhood)%.15
               neighbors = [neighbors i];
            end
        end        
    end
    
    function node = getSample(map_x,map_y,observed_map)
        if rand>.1
            node = [map_x * rand(1), map_y * rand(1)];
            if ~observed_map(ceil(node(1)),ceil(node(2)))
                node = getSample(map_x,map_y,observed_map);
            end
        else
            node=[p_goal.x p_goal.y];
        end
    end
    
    
    function indx = findNearest(rrt, n)
        mindist = norm([rrt(1).state.x rrt(1).state.y]- n);
        indx = 1;
        for i = 2:length(rrt)
            dist = norm([rrt(i).state.x rrt(i).state.y] - n);
            if (dist<mindist)
               mindist = dist;
               indx = i;
            end
        end
    end 
    
end