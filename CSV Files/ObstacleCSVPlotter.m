clear;

M = csvread('ObstacleData.csv');
sz = size(M);
num_obstacles = M(1,1);
num_samples = sz(1)-1;
for n = 1:num_obstacles
    obs_data(n).sample = [];
    obs_data(n).time = [];
    obs_data(n).x = [];
    obs_data(n).y = [];
    obs_data(n).z = [];
end
for i=2:num_samples
    num = M(i,3);
    if num ~= 0
        for j = 1:num
            obs = M(i, 3+j);
            obs_data(obs).sample = [obs_data(obs).sample M(i, 1)];
            obs_data(obs).time = [obs_data(obs).time M(i, 2)];
            obs_data(obs).x = [obs_data(obs).x M(i, 3+num+(j-1)*3 +1)];
            obs_data(obs).y = [obs_data(obs).y M(i, 3+num+(j-1)*3 +2)];
            obs_data(obs).z = [obs_data(obs).z M(i, 3+num+(j-1)*3 +3)];
        end
    end
end

lw = 3;
clf;
set(gcf, "color", "white");

subplot(2,2,1);
for i = 1: num_obstacles
    plot(obs_data(i).time, obs_data(i).x, "k", 'LineWidth', lw);
    hold on;
    text(obs_data(i).time(1),obs_data(i).x(1)+5,num2str(i), 'FontSize',12)
end
xlabel("Time (s)");
ylabel("X co-ordinate (m)");
grid on;
subplot(2,2,2);
for i = 1: num_obstacles
    plot(obs_data(i).time, obs_data(i).y, "k", 'LineWidth', lw);
    hold on;
    text(obs_data(i).time(1),obs_data(i).y(1)+5,num2str(i), 'FontSize',12)
end
xlabel("Time (s)");
ylabel("Y co-ordinate (m)");
grid on;
subplot(2,2,3);
for i = 1: num_obstacles
    plot(obs_data(i).time, obs_data(i).z, "k",'LineWidth', lw);
    hold on;
    text(obs_data(i).time(1),obs_data(i).z(1)+5,num2str(i), 'FontSize',12)
end
xlabel("Time (s)");
ylabel("Z co-ordinate (m)");
grid on;
subplot(2,2,4);
for i = 1:num_obstacles
    plot3(obs_data(i).x, obs_data(i).y, obs_data(i).z, "LineWidth", lw);
    hold on;
end
set(gca,'DataAspectRatio',[1 1 1])
grid on;
title('3D Plot of obstacles');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');