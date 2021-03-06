function [control,Q,V,road,wall,u,v] = get_fine_map(params,map_struct,observed_map,state)


maxiter=1e4;
[N,M] = size(map_struct.seed_map);

%cur_map=1-sign(1-observed_map);
cur_map=sign(observed_map);
cost_map=1-cur_map;

%road_ind_map=zeros(N,M);

[x,y] = meshgrid(-1:1, -1:1);
actions=[x(:) y(:)];
actions(5,:)=[];
%local_cost=.5*[1 sqrt(2) 1 ;sqrt(2) 0 sqrt(2) ;1 sqrt(2) 1];
%local_cost=.5*sqrt(2)*ones(3);

%blur=fspecial('disk',2);
%cost_map=1e1*conv2(cost_map,blur,'same');

%cost_map = 1e1*imgaussfilt(cost_map,.1);

action_cost=[sqrt(2) 1 sqrt(2) 1 1 sqrt(2) 1 sqrt(2)];
%action_cost(5)=[];

%n_obs=length(map_struct.bridge_locations);
%n_road=sum(sum(cur_map))-n_obs;
%n_wall=N*M-n_road-n_obs;


for ii=1:N
    for jj=1:M
        if ~cur_map(ii,jj) %0 is obs
            %wall(counter_wall,:)=[ii,jj];
            %counter_wall=counter_wall+1;
            cost_map(ii,jj)=inf;
            %{
            for kk=1:8
                indi=ii+ind(kk,1);
                indj=jj+ind(kk,2);
                if indi>0 && indi<=N &&indj>0 && indj<=M 
                    cost_map(indi,indj)=max(local_cost(2+ind(kk,1),2+ind(kk,2)),cost_map(indi,indj));
                end
            end
            %}
        else
            if sum(abs([ii ,jj]-[map_struct.goal.y map_struct.goal.x]))
                %road(counter_road,:)=[ii,jj];
                %road_ind_map(ii,jj)=counter_road;
                %counter_road=counter_road+1;
                obs_flag=zeros(8,1);
                for kk=1:8
                    indi=ii+actions(kk,1);
                    indj=jj+actions(kk,2);
                    if indi>0 && indi<=N &&indj>0 && indj<=M 
                        if ~cur_map(indi,indj)
                            obs_flag(kk)=1;
                            %cost_map(ii,jj)=cost_map(ii,jj)+local_cost(2+ind(kk,1),2+ind(kk,2));
                        end
                    end
                end
                if sum(obs_flag)
                    cost_map(ii,jj)=sqrt(.5);
                    if (obs_flag(2)&&obs_flag(7)) || (obs_flag(4)&&obs_flag(5))
                        cost_map(ii,jj)=inf;
                    end
                end
            end
        end
    end
end

for potential=1:size(map_struct.bridge_locations,2)
    p_obs=map_struct.bridge_locations(:,potential);
    cost_map(p_obs(2)-1:p_obs(2)+1,p_obs(1)-1:p_obs(1)+1)= ...
        cost_map(p_obs(2)-1:p_obs(2)+1,p_obs(1)-1:p_obs(1)+1) ...
        +1*(1-map_struct.bridge_probabilities(potential))*sqrt(.5)*ones(3);
end

[X,Y] = meshgrid(1:N, 1:M);
[Xq,Yq] = meshgrid(1:.5:N, 1:.5:M);
cur_mapq = interp2(X,Y,cur_map,Xq,Yq);
cost_mapq = interp2(X,Y,cost_map,Xq,Yq);
cur_mapq=1-sign(1-cur_mapq);

Nq=length(Xq);
Mq=length(Yq);

road=zeros(Nq*Mq,2);
wall=road;
counter_wall=0;
counter_road=0;
%road_ind_map=zeros(N,M);

%%
for ii=1:Nq
    for jj=1:Mq
        if ~cur_mapq(ii,jj) %0 is obs
            counter_wall=counter_wall+1;
            wall(counter_wall,:)=[ii,jj];
            
            %cost_map(ii,jj)=inf;
            %{
            for kk=1:8
                indi=ii+ind(kk,1);
                indj=jj+ind(kk,2);
                if indi>0 && indi<=N &&indj>0 && indj<=M 
                    cost_map(indi,indj)=max(local_cost(2+ind(kk,1),2+ind(kk,2)),cost_map(indi,indj));
                end
            end
            %}
        else
            if sum(abs([Xq(ii,jj),Yq(ii,jj)]-[map_struct.goal.x, map_struct.goal.y]))
                counter_road=counter_road+1;
                road(counter_road,:)=[ii,jj];
                %road_ind_map(ii,jj)=counter_road;
                for kk=1:8
                    indi=ii+actions(kk,1);
                    indj=jj+actions(kk,2);
                    if indi>0 && indi<=Nq &&indj>0 && indj<=Mq
                        if ~cur_mapq(indi,indj)
                            cost_mapq(ii,jj)=cost_mapq(ii,jj)+20;
                            break
                            %cost_map(ii,jj)=cost_map(ii,jj)+local_cost(2+ind(kk,1),2+ind(kk,2));
                        end
                    end
                end
            else
                ig=ii;
                jg=jj;
            end
        end
    end
end
road(counter_road+1,:)=[ig,jg];
%road_ind_map(map_struct.goal.y,map_struct.goal.x)=counter_road;
cost_mapq=7*cost_mapq;
%%


Q=inf*ones(Nq,Mq,8);
Q(ig,jg,:)=0;
V=zeros(Nq,Mq,1);

for ii=1:maxiter
    for ir=1:counter_road
        io=road(ir,1);
        jo=road(ir,2);
        for aa=1:8
            sq=[io,jo]+actions(aa,:);
            if ~cur_mapq(sq(1),sq(2))
                Q(io,jo,aa)=inf;
            else
                %{
                if (aa==1 && ~cur_map(road(ss,1)+ind(2,1),road(ss,2)+ind(2,2))&&~cur_map(road(ss,1)+ind(4,1),road(ss,2)+ind(4,2))) || ...
                        (aa==3 && ~cur_map(road(ss,1)+ind(2,1),road(ss,2)+ind(2,2))&&~cur_map(road(ss,1)+ind(5,1),road(ss,2)+ind(5,2)))|| ...
                        (aa==6 && ~cur_map(road(ss,1)+ind(4,1),road(ss,2)+ind(4,2))&&~cur_map(road(ss,1)+ind(7,1),road(ss,2)+ind(7,2)))|| ...
                        (aa==8 && ~cur_map(road(ss,1)+ind(5,1),road(ss,2)+ind(5,2))&&~cur_map(road(ss,1)+ind(7,1),road(ss,2)+ind(7,2)))
                    Q(ss,aa)=inf;
                else
                    Q(ss,aa)=action_cost(aa)+cost_map(sp(1),sp(2))+V(road_ind_map(sp(1),sp(2)));
                end
                %}
                Q(io,jo,aa)=action_cost(aa)+cost_mapq(sq(1),sq(2))+V(sq(1),sq(2));
            end
        end
    end
    [V_temp,act_ind]=min(Q,[],3);
    %V_temp=conv2(V_temp,blur,'same');
    deltaV=abs(V-V_temp);
    %{
    if mod(ii,1e2)
        max(deltaV(:))
    end
    %}
    if max(deltaV(:))<1
        break
    else
        V=V_temp;
    end
end


u=zeros(size(V));
v=u;
for io=1:Nq
    for jo=1:Mq
        if cur_mapq(io,jo) && sum(abs([Xq(io,jo),Yq(io,jo)]-[map_struct.goal.x, map_struct.goal.y]))
            u(io,jo)=actions(act_ind(io,jo),2);
            v(io,jo)=actions(act_ind(io,jo),1);
        end
    end
end

%{
%[xx,yy] = meshgrid(1:N, 1:M);
figure;surf(Xq,-Yq,V)
figure;surf(Xq,-Yq,cost_mapq)
figure;quiver(Xq(:),-Yq(:),u(:),-v(:),'r')
%figure
%surf(xx,-yy,cur_map)
close all
%}


%traj=[]
control=[];
goal_flag=0;
current_state=state;
next_position=round(2*[current_state.x current_state.y])/2;

jn=next_position(1)*2-1;
in=next_position(2)*2-1;

next_position=next_position+[u(in,jn), v(in,jn)]/2;
control_count=1;
while (~goal_flag)

    [current_state,uu,goal_flag]=my_steering(current_state, next_position,goal_flag,map_struct.goal,params,observed_map);
    control=[control uu];
    if goal_flag==2
        break% Prepare collision
    end
    control_count=control_count+1;
    
    next_position=round(2*[current_state.x current_state.y])/2;
    jn=next_position(1)*2-1;
    in=next_position(2)*2-1;
    
    next_position=next_position+[u(in,jn), v(in,jn)]/2;
    if control_count>1e3
        break
    end
end


end

