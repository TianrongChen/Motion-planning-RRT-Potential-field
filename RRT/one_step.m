%fan(1:2,:)=fan(1:2,:)-[map_struct.start.x;map_struct.start.y];
load('one_step.mat')
close all
scatter(fan(1,:),fan(2,:))
hold on
%scatter(map_struct.start.x,map_struct.start.y)
scatter(0,0)
quiver(fan(1,:),fan(2,:),.1*cos(fan(3,:)),.1*sin(fan(3,:)))

for i=1:22
    %plot([map_struct.start.x fan(1,i)],[map_struct.start.y fan(2,i)],'b-')
    plot([0 fan(1,i)],[0 fan(2,i)],'b-')
end

figure
plot(fan(3,3:end)-fan(3,2:end-1))
%axis equal