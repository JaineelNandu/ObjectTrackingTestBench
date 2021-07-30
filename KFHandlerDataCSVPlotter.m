clear;
K = csvread('KFHandlerData.csv');
sz_k = size(K);
num_tracked_obstacles = K(1,1);
max_size = K(1,2);
num_tracked_samples = sz_k(1);
num_tr = [];
num_possible = [];
t = [];
for n = 1:num_tracked_obstacles
    tracked_data(n).time = [];
    tracked_data(n).x = [];
    tracked_data(n).y = [];
    tracked_data(n).z = [];
end
for i=2:num_tracked_samples
    num = K(i,4);
    num_tr = [num_tr num];
    num_possible = [num_possible K(i, 3)];
    t = [t K(i, 2)];
    if num ~= 0
        for j = 1:num
            obs = K(i, 4+j);
                tracked_data(obs).time = [tracked_data(obs).time K(i, 2)];
            tracked_data(obs).x = [tracked_data(obs).x K(i, 4+num+(j-1)*6 +1)];
            tracked_data(obs).y = [tracked_data(obs).y K(i, 4+num+(j-1)*6 +2)];
            tracked_data(obs).z = [tracked_data(obs).z K(i, 4+num+(j-1)*6 +3)];
            
        end
    end
end

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
        end
    end
end

lw1 = 2;
lw2 = 4;
clf;
set(gcf, "color", "white");
hold on;
subplot(2,2,1);
for i = 1: num_passed_obstacles
    stairs(sen_data(i).time, sen_data(i).x, "k", 'LineWidth', lw1);
    hold on;
    if any(sen_data(i).time)
        text(sen_data(i).time(1),sen_data(i).x(1)+5,num2str(i), 'FontSize',12)
    end
end
xlabel("Time (s)");
ylabel("X co-ordinate (m)");
grid on;
subplot(2,2,2);
for i = 1: num_passed_obstacles
    stairs(sen_data(i).time, sen_data(i).y, "k",'LineWidth', lw1);
    hold on;
    if any(sen_data(i).time)
        text(sen_data(i).time(1),sen_data(i).y(1)+5,num2str(i), 'FontSize',12)
    end
end
xlabel("Time (s)");
ylabel("Y co-ordinate (m)");
grid on;
subplot(2,2,3);
for i = 1: num_passed_obstacles
    stairs(sen_data(i).time, sen_data(i).z, "k",'LineWidth', lw1);
    hold on;
    if any(sen_data(i).time)
        text(sen_data(i).time(1),sen_data(i).z(1)+5,num2str(i), 'FontSize',12)
    end
end
xlabel("Time (s)");
ylabel("Z co-ordinate (m)");
grid on;

hold on;

subplot(2,2,1);
for i = 1: num_tracked_obstacles
    stem(tracked_data(i).time, tracked_data(i).x, 'filled', "MarkerSize", lw2);
    %stairs(tracked_data(i).time, tracked_data(i).x, 'LineWidth', lw2);
    hold on;
end
xlabel("Time (s)");
ylabel("X co-ordinate (m)");
grid on;
subplot(2,2,2);
for i = 1: num_tracked_obstacles
    stem(tracked_data(i).time, tracked_data(i).y, 'filled', "MarkerSize", lw2);
    %stairs(tracked_data(i).time, tracked_data(i).y, 'LineWidth', lw2);
    hold on;
end
xlabel("Time (s)");
ylabel("Y co-ordinate (m)");
grid on;
subplot(2,2,3);
for i = 1: num_tracked_obstacles
    stem(tracked_data(i).time, tracked_data(i).z, 'filled', "MarkerSize", lw2);
    %stairs(tracked_data(i).time, tracked_data(i).z, 'LineWidth', lw2);
    hold on;
end
xlabel("Time (s)");
ylabel("Z co-ordinate (m)");
grid on;
subplot(2,2,4)
stairs(t, num_possible, ":b", "LineWidth",lw1*1.5);
hold on;
stairs(t, num_tr, "g","LineWidth",lw1);
yticks([0:1:max_size+1]);
ylim([0, max_size+1]);
yline(max_size, "r", "LineWidth", lw1);
xlabel("Time (s)");
ylabel("Number of Obstacles");
legend("Requested to handler","tracked", "handler capacity");