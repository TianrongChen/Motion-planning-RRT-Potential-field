function result = myPlanPathRRTstar(params,rrts_param, cur_state,observed_map, p_goal)

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
        for ii=1:M % try M samples
            sample_node = getSample(map_x,map_y,observed_map);
            nearest_node_ind = findNearest(rrt, sample_node);
            [new_state, parent_control, flags] = steering(rrt(nearest_node_ind).state, sample_node,rrt(nearest_node_ind).goalReached);
       
            if (flags~=2)% no collision
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
                if flags==1
                    break;
                end
                new_neighbors_ind = [];%getNeighbors(rrt, new_state);
                if (~isempty(new_neighbors_ind))
                    rrt = reWire(rrt, new_neighbors_ind, parent_node_ind, length(rrt));
                end
                
            end
            if ~mod(ii,50)
%                 disp sample node
                disp(['sample: ' num2str(ii) ' node: ' num2str(j)])
            end
            
            if j>N
                break
            end
        end
        figure(2)
        axis([0 50 0 50])
        hold on;
        setPath(rrt);
%         text1 = strcat('Total number of generated nodes:', num2str(j-1))
%         text1 = strcat('Total number of nodes in tree:', length(rrt))
    end


    function setPath(rrt)
        % plot the tree
%         plotx=zeros(1,length(rrt));
%         ploty=plotx;
        for i = 1: length(rrt)
%             plotx(i)=rrt(i).state.x;
%             ploty(i)=rrt(i).state.y;
            
            p1 = [rrt(i).state.x rrt(i).state.y];
            rob.x = p1(1); rob.y=p1(2);
            figure(2)
            set(gca,'Ydir','reverse');
            plot(rob.x,rob.y,'.b')
            hold on
            for j = 1: length(rrt(i).ichild)
                p2 = [rrt(rrt(i).ichild(j)).state.x rrt(rrt(i).ichild(j)).state.y];
                figure(2)
                plot([p1(1),p2(1)], [p1(2),p2(2)], 'b', 'LineWidth', 1);
                hold on
            end
            
        end
        
%         figure(2)
%         set(gca,'Ydir','reverse');
%         plot(plotx,ploty,'.b')
%         hold on
        
        [cost,i] = getFinalResult(rrt);
        result.cost = cost;
        result.rrt = rrt;
        final_control=[];
        
        % for control result
        p11=[];
        while i ~= 0
            p11 = [p11 [rrt(i).state.x ;rrt(i).state.y]];
            final_control=[rrt(i).parent_control final_control];
            i = rrt(i).iParent;
        end  
        result.control=final_control;
        result.time_taken = toc;
        figure(2)
        % final path
        plot(p11(1,:),p11(2,:),'b', 'Marker','.', 'MarkerSize', 30);
        hold on
        set(gca,'Ydir','reverse');
        
    end

    %yunzhi
    % find the best value and corresponding min_node_ind
    function [value,min_node_ind] = getFinalResult(rrt)
        goal_ind = find([rrt.goalReached]==1);
        if ~(isempty(goal_ind))
            % success
            disp('Possible way is found!');
            rrt_goal = rrt(goal_ind);
            [value,min_node_ind] = min(rrt_goal.moves);
            min_node_ind=goal_ind(min_node_ind);
            %u=
        else
            % failure
            disp('Could not find the possible way!');
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
        go=round(min(floor(norm(dp)/.15),5*rrts_param.neighbourhood/.15));
        theta_new=atan2(dp(2),dp(1));
        dtheta=theta_new-nearest_state.theta;
        left_right=sign(dtheta);
        angle=abs(dtheta);
        
        turn=floor(angle/rrts_param.maxturn);%.05
        res=angle-turn*rrts_param.maxturn;
        u=[-2*ones(1,4*turn),left_right*res/rrts_param.maxturn,zeros(1,go)];
        ind=1:(4*turn);
        u(mod(ind,4)==3)=left_right;
        
        %run this u
        [new_state, ~, flags,ind] = run_actions(u,params, nearest_state, observed_map, p_goal,goal_flag);
 
        if flags==2 && ind>40
            u=u(1:ind);
            [new_state, ~, flags] = run_actions(u,params, nearest_state, observed_map, p_goal,goal_flag);
        end
    end

    function [state, observed_map, flags,ind] = run_actions(u,params, state, observed_map, target,flags)
        ind=length(u);
        for i = 1:ind
            if (state.moveCount < params.max_moveCount && flags ~= 2)
                action=u(i);

                [state, observed_map, flags] = SIM_motionModel(params, state, action, observed_map, observed_map, target);
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
    
%% insert code
    function rrt = insertNode(rrt, parent, new_node,parent_control,flag)
        rrt(end+1) = struct(field1, new_node, field2, parent, field3, parent_control, field4, [],field5, new_node.moveCount,field6,flag);
        rrt(parent).ichild=[rrt(parent).ichild length(rrt)];
    end
    
%% chooseParent
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

    %% biased sample
    function node = getSample(map_x,map_y,observed_map)
        if rand>.05
            node = [map_x * rand(1), map_y * rand(1)];
            % if wall, sample again
            if ~observed_map(ceil(node(1)),ceil(node(2)))
                node = getSample(map_x,map_y,observed_map);
            else
                flag=0;
                while flag~=1 
                    mu=node;
                    % hyper parameter
                    sigma = [1 0; 0 1];
                    temp_node=mvnrnd(mu,sigma);
                    
                    if (ceil(temp_node(1))<1 || ceil(temp_node(1))>49 || ceil(temp_node(2))<1 || ceil(temp_node(2))>49)
                        continue
                    end
                    flag=observed_map(ceil(temp_node(1)),ceil(temp_node(2)));
                    if(flag==1)
                        new_node=(temp_node+node)/2;
                        if (ceil(new_node(1))<1 || ceil(new_node(1))>49 || ceil(new_node(2))<1 || ceil(new_node(2))>49)
                            continue
                        end
                        flag=observed_map(ceil(new_node(1)),ceil(new_node(2)));
                        if(flag==1)
                            node=new_node;
                            return
                        end
                    end
                end
                
            end
        else
            node=[p_goal.x p_goal.y];
        end
    end
    
    %%  find nearest
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