function [control,Q,V,road,wall,road_ind_map,u,v] = get_map(params,map_struct,observed_map,state)



start();

    function start()
        maxiter=1e3;
        [N,M] = size(map_struct.seed_map);

        cur_map=sign(observed_map);
        cost_map=zeros(size(cur_map));

        road_ind_map=zeros(N,M);

        [x,y] = meshgrid(-1:1, -1:1);
        ind=[x(:) y(:)];
        ind(5,:)=[];
        local_cost=1e1*[1 sqrt(2) 1 ;sqrt(2) 0 sqrt(2) ;1 sqrt(2) 1]/2;

        action_cost=local_cost(:);
        action_cost(5)=[];

        n_obs=length(map_struct.bridge_locations);
        n_road=sum(sum(cur_map))-n_obs;
        n_wall=N*M-n_road-n_obs;
        road=zeros(n_road+n_obs,2);
        wall=zeros(n_wall,2);
        counter_wall=1;
        counter_road=1;

        for ii=1:N
            for jj=1:M
                if ~cur_map(ii,jj) %0 is obs
                    wall(counter_wall,:)=[ii,jj];
                    counter_wall=counter_wall+1;
                    cost_map(ii,jj)=inf;
                    %{
                    for kk=1:8
                        indi=ii+ind(kk,1);
                        indj=jj+ind(kk,2);
                        if indi>0 && indi<=N &&indj>0 && indj<=M 
                            cost_map(indi,indj)=min(local_cost(2+ind(kk,1),2+ind(kk,2)),cost_map(indi,indj));
                        end
                    end
                    %}
                else
                    if sum(abs([ii ,jj]-[map_struct.goal.y map_struct.goal.x]))
                        road(counter_road,:)=[ii,jj];
                        road_ind_map(ii,jj)=counter_road;
                        counter_road=counter_road+1;
                        obs_flag=zeros(8,1);
                        for kk=1:8
                            indi=ii+ind(kk,1);
                            indj=jj+ind(kk,2);
                            if indi>0 && indi<=N &&indj>0 && indj<=M 
                                if ~cur_map(indi,indj)
                                    obs_flag(kk)=1;
                                    cost_map(ii,jj)=cost_map(ii,jj)+local_cost(2+ind(kk,1),2+ind(kk,2));
                                end
                            end
                        end
                        if (obs_flag(2)&&obs_flag(7)) || (obs_flag(4)&&obs_flag(5))
                            cost_map(ii,jj)=inf;
                        end
                    end
                end
            end
        end

        road(counter_road,:)=[map_struct.goal.y,map_struct.goal.x];
        road_ind_map(map_struct.goal.y,map_struct.goal.x)=counter_road;

        %cost_map=(1-cost_map)*2;

        Q=zeros(n_road+n_obs,8);
        V=zeros(n_road+n_obs,1);

        for ii=1:maxiter
            for ss=1:(n_road+n_obs-1)
                for aa=1:8
                    sp=road(ss,:)+ind(aa,:);
                    if ~cur_map(sp(1),sp(2))
                        Q(ss,aa)=inf;
                    else
                        if (aa==1 && ~cur_map(road(ss,1)+ind(2,1),road(ss,2)+ind(2,2))&&~cur_map(road(ss,1)+ind(4,1),road(ss,2)+ind(4,2))) || ...
                                (aa==3 && ~cur_map(road(ss,1)+ind(2,1),road(ss,2)+ind(2,2))&&~cur_map(road(ss,1)+ind(5,1),road(ss,2)+ind(5,2)))|| ...
                                (aa==6 && ~cur_map(road(ss,1)+ind(4,1),road(ss,2)+ind(4,2))&&~cur_map(road(ss,1)+ind(7,1),road(ss,2)+ind(7,2)))|| ...
                                (aa==8 && ~cur_map(road(ss,1)+ind(5,1),road(ss,2)+ind(5,2))&&~cur_map(road(ss,1)+ind(7,1),road(ss,2)+ind(7,2)))
                            Q(ss,aa)=inf;
                        else
                            Q(ss,aa)=action_cost(aa)+cost_map(sp(1),sp(2))+V(road_ind_map(sp(1),sp(2)));
                        end
                    end
                end
            end
            [V_temp,act_ind]=min(Q,[],2);
            if max(abs(V-V_temp))<1e-6
                break
            else
                V=V_temp;
            end
        end


        %[xx,yy] = meshgrid(1:N, 1:M);
        u=zeros(size(V));
        v=u;

        VV=zeros(N,M);
        for ijk=1:(n_road+n_obs)
            VV(road(ijk,1),road(ijk,2))=V(ijk);
            u(ijk)=ind(act_ind(ijk),1);
            v(ijk)=ind(act_ind(ijk),2);
        end


        %{
        [xx,yy] = meshgrid(1:N, 1:M);
        figure
        surf(xx,-yy,VV)
        figure
        surf(xx,-yy,cost_map)
        figure
        quiver(road(:,2),-road(:,1),v,-u,'r')
        figure
        surf(xx,-yy,cur_map)
        %}


        %traj=[]
        control=[];
        goal_flag=0;
        current_state=state;
        next_position=[current_state.x current_state.y];

        while (~goal_flag)

            [current_state,uu,goal_flag]=steering(current_state, next_position,goal_flag);
            control=[control uu];
            if goal_flag==2
                break% Prepare collision
            end
            next_position=next_position+[v(road_ind_map(next_position(1),next_position(2))), ...
                u(road_ind_map(next_position(1),next_position(2)))];
        end
    end

    function [new_state,u,flags]=steering(current_state, next_position,goal_flag)
        dp=next_position-[current_state.x, current_state.y];
        %go=round(min(floor(norm(dp)/.15),1*rrts_param.neighbourhood/.15));
        go=round(norm(dp)/.15);
        theta_new=atan2(dp(2),dp(1));

        dtheta=theta_new-current_state.theta;

        left_right=sign(dtheta);
        angle=abs(dtheta);
        %%{
        if angle>pi
            angle=pi*2-angle;
            left_right=-left_right;
        end
        %}
        turn=floor(angle/.05);%.05
        res=angle-turn*.05;
        %%{
        u=[-2*ones(1,4*turn),left_right*res/.05,zeros(1,go)];
        control_ind=1:(4*turn);
        u(mod(control_ind,4)==0)=left_right;
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

        %%{
        %run this u
        p_goal=map_struct.goal;
        [new_state, ~, flags,control_ind] = run_actions(u,params, current_state, observed_map, p_goal,goal_flag);
        if flags==2 && control_ind>0
            u=u(1:control_ind);
            [new_state, ~, flags] = run_actions(u,params, current_state, observed_map, p_goal,goal_flag);
        end
        if flags==2
            fprintf('I am sorry! This control will lead to collision!!!\n')
        end
        %}
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
                    ind=i-4;
                    break
                end
            end
        end
    end
    

end

