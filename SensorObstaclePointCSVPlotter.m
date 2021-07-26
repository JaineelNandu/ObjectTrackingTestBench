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

O = csvread('SensorObstacleParamData.csv');
azimuthal = O(1,2);
elevation = O(1,3);
r_min = O(1,4);
r_max = O(1,5);
T = [O(1,6), O(1,7), O(1,8), O(1,9);
     O(1,10), O(1,11), O(1,12), O(1,13);
     O(1,14), O(1,15), O(1,16), O(1,17);
     O(1,18), O(1,19), O(1,20), O(1,21)];

lw1 = 2;
lw2 = 4;
clf;
set(gcf, "color", "white");

subplot(2,2,1);
for i = 1: num_obstacles
    plot(obs_data(i).time, obs_data(i).x, ":k", 'LineWidth', lw1);
    hold on;
    text(obs_data(i).time(1),obs_data(i).x(1)+5,num2str(i), 'FontSize',12)
end
xlabel("Time (s)");
ylabel("X co-ordinate (m)");
grid on;
subplot(2,2,2);
for i = 1: num_obstacles
    plot(obs_data(i).time, obs_data(i).y, ":k", 'LineWidth', lw1);
    hold on;
    text(obs_data(i).time(1),obs_data(i).y(1)+5,num2str(i), 'FontSize',12)
end
xlabel("Time (s)");
ylabel("Y co-ordinate (m)");
grid on;
subplot(2,2,3);
for i = 1: num_obstacles
    plot(obs_data(i).time, obs_data(i).z, ":k",'LineWidth', lw1);
    hold on;
    text(obs_data(i).time(1),obs_data(i).z(1)+5,num2str(i), 'FontSize',12)
end
xlabel("Time (s)");
ylabel("Z co-ordinate (m)");
grid on;
subplot(2,2,4);
for i = 1:num_obstacles
    plot3(obs_data(i).x, obs_data(i).y, obs_data(i).z, ":k", "LineWidth", lw1);
    hold on;
end
set(gca,'DataAspectRatio',[1 1 1])
grid on;
title('3D Plot of obstacles');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

hold on;
col = 'k';
lw = 4;

set(gcf, "color", "white");
subplot(2,2,1);
for i = 1: num_passed_obstacles
    stairs(sen_data(i).time, sen_data(i).x, 'LineWidth', lw2);
    hold on;
end
xlabel("Time (s)");
ylabel("X co-ordinate (m)");
grid on;
subplot(2,2,2);
for i = 1: num_passed_obstacles
    stairs(sen_data(i).time, sen_data(i).y, 'LineWidth', lw2);
    hold on;
end
xlabel("Time (s)");
ylabel("Y co-ordinate (m)");
grid on;
subplot(2,2,3);
for i = 1: num_passed_obstacles
    stairs(sen_data(i).time, sen_data(i).z, 'LineWidth', lw2);
    hold on;
end
xlabel("Time (s)");
ylabel("Z co-ordinate (m)");
grid on;
subplot(2,2,4);
SenX = [0 5; 0 0; 0 0; 1 1];
SenY = [0 0; 0 5; 0 0; 1 1];
SenZ = [0 0; 0 0; 0 5; 1 1];
SenX = eye(4)*SenX;
SenY = eye(4)*SenY;
SenZ = eye(4)*SenZ;
plot3([SenX(1,:)]', [SenX(2,:)]', [SenX(3,:)]', 'r', "LineWidth",lw);
hold on;
plot3([SenY(1,:)]',[SenY(2,:)]', [SenY(3,:)]', 'g', 'LineWidth',lw)
plot3([SenZ(1,:)]',[SenZ(2,:)]', [SenZ(3,:)]', 'b', 'LineWidth',lw)
hold on;
T = inv(T);
            %T = eye(4); 
            el1 = linspace((-elevation/2)*(pi/180), (elevation/2)*(pi/180), 30);
            r1 = r_min*ones(1, length(el1));
            el2 = fliplr(el1);
            el_1 = [el1 el2];
            r1 = [r1 r_max*ones(1,length(el1))];
            azim1 = -((azimuthal/2)*(pi/180))*ones(1, length(el_1));
            [X1, Y1, Z1] = sph2cart(azim1, el_1, r1);
            a1 = T*[X1; Y1; Z1;ones(1, length(X1))];
            f(1).x = a1(1,:)';
            f(1).y = a1(2,:)';
            f(1).z = a1(3,:)';
            azim2 = -azim1;
            [X2, Y2, Z2] = sph2cart(azim2, el_1,r1);
            a2 = T*[X2; Y2; Z2; ones(1, length(X2))];
            f(2).x = a2(1,:)';
            f(2).y = a2(2,:)';
            f(2).z = a2(3,:)';

            azim3 = linspace((-azimuthal/2)*(pi/180), (azimuthal/2)*(pi/180), 30);
            %r3 = rmin*ones(1,length(azim3));
            azim4 = fliplr(azim3);
            azim_2 = [azim3 azim4];
            %r = [r3 rmax*ones(1,length(azim_2))];
            el3 = -((elevation/2)*(pi/180))*ones(1, length(azim_2));
            [X3, Y3, Z3] = sph2cart(azim_2, el3, r1);
            a3 = T*[X3; Y3; Z3; ones(1, length(X3))];
            f(3).x = a3(1,:)';
            f(3).y = a3(2,:)';
            f(3).z = a3(3,:)';
            el4 = -el3;
            [X4, Y4, Z4] = sph2cart(azim_2, el4, r1);
            a4 = T*[X4; Y4; Z4; ones(1, length(X4))];
            f(4).x = a4(1,:)';
            f(4).y = a4(2,:)';
            f(4).z = a4(3,:)';
            
            azim = [azim1 azim3 ([azim2 azim4])];
            el = [el1 el3 ([el2 el4])];
            r = r_max*ones(1, length(azim));
            [X5, Y5, Z5] = sph2cart(azim, el, r);
            a5 = T*[X5; Y5; Z5; ones(1, length(X5))];
            f(5).x = a5(1,:)';
            f(5).y = a5(2,:)';
            f(5).z = a5(3,:)';
            falpha = 0.05;
            fill3(f(1).x, f(1).y, f(1).z,col ,'FaceAlpha', falpha);
            fill3(f(2).x, f(2).y, f(2).z,col ,'FaceAlpha', falpha);
            fill3(f(3).x, f(3).y, f(3).z,col ,'FaceAlpha', falpha);
            fill3(f(4).x, f(4).y, f(4).z,col ,'FaceAlpha', falpha);
for i = 1:num_passed_obstacles
    plot3(sen_data(i).x, sen_data(i).y, sen_data(i).z, "LineWidth", lw2);
    hold on;
end
set(gca,'DataAspectRatio',[1 1 1])
grid on;
title('3D Plot of obstacles in ego vehicle frame');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
