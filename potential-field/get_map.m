function [control,Q,V,road,wall,road_ind_map,u,v] = get_map(params,map_struct,observed_map,state)


maxiter=1e3;
[N,M] = size(map_struct.seed_map);

cur_map=sign(observed_map);
cost_map=1-cur_map;

road_ind_map=zeros(N,M);

[x,y] = meshgrid(-1:1, -1:1);
ind=[x(:) y(:)];
ind(5,:)=[];
%local_cost=.5*[1 sqrt(2) 1 ;sqrt(2) 0 sqrt(2) ;1 sqrt(2) 1];
%local_cost=.5*sqrt(2)*ones(3);

%blur=fspecial('disk',2);
%cost_map=1e1*conv2(cost_map,blur,'same');

%cost_map = 1e1*imgaussfilt(cost_map,.1);

action_cost=[sqrt(2) 1 sqrt(2) 1 1 sqrt(2) 1 sqrt(2)];
%action_cost(5)=[];

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
                    cost_map(indi,indj)=max(local_cost(2+ind(kk,1),2+ind(kk,2)),cost_map(indi,indj));
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

for potential=1:length(map_struct.bridge_locations)
    p_obs=map_struct.bridge_locations(:,potential);
    cost_map(p_obs(2)-1:p_obs(2)+1,p_obs(1)-1:p_obs(1)+1)= ...
        cost_map(p_obs(2)-1:p_obs(2)+1,p_obs(1)-1:p_obs(1)+1) ...
        +1*(1-map_struct.bridge_probabilities(potential))*sqrt(.5)*ones(3);
end


road(counter_road,:)=[map_struct.goal.y,map_struct.goal.x];
road_ind_map(map_struct.goal.y,map_struct.goal.x)=counter_road;

%cost_map=(1-cost_map)*2;
cost_map=7*cost_map;

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
    %V_temp=conv2(V_temp,blur,'same');
    if max(abs(V-V_temp))<1e-6
        break
    else
        V=V_temp;
    end
end

%[Xq,Yq,Zq] = meshgrid(1:.5:N,-3:.25:3,-3:.25:3);

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
%figure
%surf(xx,-yy,cur_map)
close all
%}


%traj=[]
control=[];
goal_flag=0;
current_state=state;
next_position=round([current_state.x current_state.y]);
next_position=next_position ...
    +[v(road_ind_map(next_position(2),next_position(1))), ...
        u(road_ind_map(next_position(2),next_position(1)))];

while (~goal_flag)

    [current_state,uu,goal_flag]=my_steering(current_state, next_position,goal_flag,map_struct.goal,params,observed_map);
    control=[control uu];
    if goal_flag==2
        break% Prepare collision
    end
    next_position=round([current_state.x current_state.y]);
    next_position=next_position+[v(road_ind_map(next_position(2),next_position(1))), ...
        u(road_ind_map(next_position(2),next_position(1)))];
end


    

    
    

end

