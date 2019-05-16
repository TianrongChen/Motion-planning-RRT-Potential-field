
function [state, observed_map, flags,ind] = my_run_actions(u,params, state, observed_map, goal,flags)
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
                ind=i-1;
                break
            end
        end
    end
end