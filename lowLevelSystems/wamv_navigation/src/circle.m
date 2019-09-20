close all;
clear all; 
clc;

% x = [22.4645, 26, 29.5355, 31, 29.5355, 26, 22.4645, 21, 22.4645];
% y = [19.5355, 21, 19.5355, 16, 12.4645, 11, 12.4645, 16, 19.5355];
% plot(x, y);

points = [31, 16; 29.5355, 19.5355; 26, 21; 22.4645, 19.5355; 21, 16; 22.4645, 12.4645; 26, 11; 29.5355, 12.4645; 31, 16];
x = points(:,1);
y = points(:,2);
plot(x,y);
hold on

vehicle_pos = [10.199, 16.70003];
target_pos = [26. 16];
smallest = [21. 16];
sec_smallest = [22.4645,19.5355];

plot([vehicle_pos(1), target_pos(1)], [vehicle_pos(2), target_pos(2)]);
plot([vehicle_pos(1), smallest(1)], [vehicle_pos(2), smallest(2)]);
plot([vehicle_pos(1), sec_smallest(1)], [vehicle_pos(2), sec_smallest(2)]);

axis equal;
hold off











