clear
load reachable_x_y;
low = reachable_x_y;
load approachable_x_y;
high = approachable_x_y;
figure(1)
plot(reachable_x_y(:,1),reachable_x_y(:,2),'*',approachable_x_y(:,1),approachable_x_y(:,2),'kx')