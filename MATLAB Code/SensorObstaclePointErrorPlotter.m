N = csvread('SensorObstaclePointData.csv');
sz = size(N);
num_passed_obstacles = N(1,1);
sigmaX = N(1,2);
sigmaY = N(1,3);
sigmaZ = N(1,4);
num_samples = sz(1);
for n = 1:num_passed_obstacles
    sen_data(n).time = [];
    sen_data(n).x = [];
    sen_data(n).y = [];
    sen_data(n).z = [];
    sen_data(n).xtruth = [];
    sen_data(n).ytruth = [];
    sen_data(n).ztruth = [];
end
for i=2:num_samples
    num = N(i,3);
    if num ~= 0
        for j = 1:num
            obs = N(i, 3+j);
            sen_data(obs).time = [sen_data(obs).time N(i, 2)];
            sen_data(obs).x = [sen_data(obs).x N(i, 3+num+(j-1)*6 +1)];
            sen_data(obs).y = [sen_data(obs).y N(i, 3+num+(j-1)*6 +2)];
            sen_data(obs).z = [sen_data(obs).z N(i, 3+num+(j-1)*6 +3)];
            sen_data(obs).xtruth = [sen_data(obs).xtruth N(i, 3+num+(j-1)*6 +4)];
            sen_data(obs).ytruth = [sen_data(obs).ytruth N(i, 3+num+(j-1)*6 +5)];
            sen_data(obs).ztruth = [sen_data(obs).ztruth N(i, 3+num+(j-1)*6 +6)];
        end
    end
end

lw1 = 3;

clf;
subplot(3,1,1);
for i = 1: num_passed_obstacles
    if(any(sen_data(i).x))
    stairs(sen_data(i).time, sen_data(i).xtruth - sen_data(i).x, 'LineWidth', lw1);
    hold on;
    text(sen_data(i).time(1)-0.05,sen_data(i).xtruth(1) - sen_data(i).x(1), num2str(i), 'FontSize',12)
    end
end
title ("Error due to noise in X axis");
xlabel("Time (s)");
ylabel("Distance (m)");
grid on;

subplot(3,1,2);
for i = 1: num_passed_obstacles
    if(any(sen_data(i).x))
    stairs(sen_data(i).time, sen_data(i).ytruth - sen_data(i).y, 'LineWidth', lw1);
    hold on;
    text(sen_data(i).time(1)-0.05,sen_data(i).ytruth(1) - sen_data(i).y(1), num2str(i), 'FontSize',12)
    end
end
title ("Error due to noise in Y axis");
xlabel("Time (s)");
ylabel("Distance (m)");
grid on;

subplot(3,1,3);
for i = 1: num_passed_obstacles
    if(any(sen_data(i).x))
    stairs(sen_data(i).time, sen_data(i).ztruth - sen_data(i).z, 'LineWidth', lw1);
    hold on;
    text(sen_data(i).time(1)-0.05,sen_data(i).ztruth(1) - sen_data(i).z(1), num2str(i), 'FontSize',12)
    end
end
title ("Error due to noise in Z axis");
xlabel("Time (s)");
ylabel("Distance (m)");
grid on;