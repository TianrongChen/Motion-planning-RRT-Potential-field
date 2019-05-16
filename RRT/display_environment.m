figure(1);
if (DISPLAY_TYPE)
    imshow(imresize(observed_map, scale, 'nearest'),'Border','tight');
else
    ind = find(observed_map == 0);
    plot(x(ind)*scale,y(ind)*scale,'k.','MarkerSize', .5*scale);
end
hold on;    
plot(scale*map_struct.start.x,scale*map_struct.start.y,'g.', 'MarkerSize', 2*scale);
plot(scale*map_struct.goal.x,scale*map_struct.goal.y,'r.', 'MarkerSize', 2*scale);
line(scale*[state.border(1,:); state.border(1,[2:end 1])], scale*[state.border(2,:); state.border(2,[2:end 1])], 'Color','Red', 'LineSmoothing', 'on');
plot(scale*state.x,scale*state.y,'b.', 'MarkerSize', 2*scale);
line(scale*[state.x,state.x+params.length/2*cos(state.theta)]',scale*[state.y,state.y+params.length/2*sin(state.theta)]','Color','Blue', 'LineSmoothing', 'on');

axis equal;
axis([0 500 0 500]);
axis off;
hold off;