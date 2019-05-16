function [new_state,u,flags]=my_steering(current_state, next_position,goal_flag,p_goal,params,observed_map)
    dp=next_position-[current_state.x, current_state.y];
    %go=round(min(floor(norm(dp)/.15),1*rrts_param.neighbourhood/.15));
    %wrong=current_state.theta/2/pi;
    
    go=floor(norm(dp)/.15);
    theta_new=atan2(dp(2),dp(1));

    dtheta=theta_new-wrapToPi(current_state.theta);

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
    u(mod(control_ind,4)==1)=left_right;
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
    %p_goal=map_struct.goal;
    [new_state, ~, flags,control_ind] = my_run_actions(u,params, current_state, observed_map, p_goal,goal_flag);
    
    if flags==2 && control_ind>0
        u=u(1:control_ind);
        [new_state, ~, flags] = my_run_actions(u,params, current_state, observed_map, p_goal,goal_flag);
    end
    
    %%{
    if flags==2
        angle=pi*2-angle;
        left_right=-left_right;
        
        turn=floor(angle/.05);%.05
        res=angle-turn*.05;
        
        u=[-2*ones(1,4*turn),left_right*res/.05,zeros(1,go)];
        control_ind=1:(4*turn);
        u(mod(control_ind,4)==1)=left_right;
        
        [new_state, ~, flags] = my_run_actions(u,params, current_state, observed_map, p_goal,goal_flag);
    end
    %}
    
    %%{
    if flags==2
        go=floor(norm(dp)/.05);

        dtheta=theta_new-wrapToPi(current_state.theta+pi);

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
        u=[-2*ones(1,4*turn),left_right*res/.05,-2*ones(1,go)];
        control_ind=1:(4*turn);
        u(mod(control_ind,4)==1)=left_right;
        
        %{
        turn=floor(angle/.05);%.05
        res=angle-turn*.05;
        
        u=[-2*ones(1,4*turn),left_right*res/.05,zeros(1,go)];
        control_ind=1:(4*turn);
        u(mod(control_ind,4)==1)=left_right;
        %}
        [new_state, ~, flags] = my_run_actions(u,params, current_state, observed_map, p_goal,goal_flag);
    end
    %}
    
    %%{
    %map3 before -2
    if flags==2
        
        u=[0 0 0 0];
        
        [new_state, ~, flags] = my_run_actions(u,params, current_state, observed_map, p_goal,goal_flag);
    end
    %}
    
    %%{
    if flags==2
        
        u=-2;
        
        [new_state, ~, flags] = my_run_actions(u,params, current_state, observed_map, p_goal,goal_flag);
    end
    %}
    %%{
    if flags==2
        
        u=-1;
        
        [new_state, ~, flags] = my_run_actions(u,params, current_state, observed_map, p_goal,goal_flag);
    end
    %}
    %{
    if flags==2
        
        u=rand()*2-1;
        
        [new_state, ~, flags] = my_run_actions(u,params, current_state, observed_map, p_goal,goal_flag);
    end
    %}
    if flags==2
        fprintf('I am sorry! This control will lead to collision!!!\n')
    end
    %}
end