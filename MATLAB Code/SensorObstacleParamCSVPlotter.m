%clear;

M = csvread('SensorObstacleParamData.csv');
sz = size(M);
num_obstacles = M(1,1);
azimuthal = M(1,2);
elevation = M(1,3);
r_min = M(1,4);
r_max = M(1,5);
%T = [M(1,6), M(1,7), M(1,8), M(1,9);
%     M(1,10), M(1,11), M(1,12), M(1,13);
%     M(1,14), M(1,15), M(1,16), M(1,17);
%     M(1,18), M(1,19), M(1,20), M(1,21)];
num_samples = sz(1);
for n = 1:num_obstacles
    sen_param(n).sample = [];
    sen_param(n).time = [];
    sen_param(n).azim = [];
    sen_param(n).elev = [];
    sen_param(n).r = [];
end
for i=2:num_samples
    num = M(i,3);
    if num ~= 0
        for j = 1:num
            obs = M(i, 3+j);
            sen_param(obs).sample = [sen_param(obs).sample M(i, 1)];
            sen_param(obs).time = [sen_param(obs).time M(i, 2)];
            sen_param(obs).azim = [sen_param(obs).azim M(i, 3+num+(j-1)*3 +1)];
            sen_param(obs).elev = [sen_param(obs).elev M(i, 3+num+(j-1)*3 +2)];
            sen_param(obs).r = [sen_param(obs).r M(i, 3+num+(j-1)*3 +3)];
        end
    end
end


lw1 = 3;
lw2 = 2;
f2 = figure;
clf;
set(gcf, "color", "white");


subplot(2,2,1);
for i = 1: num_obstacles
    stairs(sen_param(i).time, sen_param(i).azim, 'LineWidth', lw1);
    hold on;
    if (any(sen_param(i).time))
    text(sen_param(i).time(1),sen_param(i).azim(1)+5,num2str(i), 'FontSize',12)
    end
end
yline(-azimuthal/2, "--k", "LineWidth", lw2);
yline(azimuthal/2, "--k", "LineWidth", lw2);
yticks([-180:60:180]);
xlabel("Time (s)");
ylabel("Azimuth angle (degrees)");
grid on;
subplot(2,2,2);
for i = 1: num_obstacles
    stairs(sen_param(i).time, sen_param(i).elev, 'LineWidth', lw1);
    hold on;
    if (any(sen_param(i).time))
    text(sen_param(i).time(1),sen_param(i).elev(1)+5,num2str(i), 'FontSize',12)
    end
end
yline(-elevation/2, "--k", "LineWidth", lw2);
yline(elevation/2, "--k", "LineWidth", lw2);
xlabel("Time (s)");
ylabel("Elevation angle (degrees)");
yticks([-180:10:180]);
grid on;
subplot(2,2,3);
for i = 1: num_obstacles
    stairs(sen_param(i).time, sen_param(i).r,'LineWidth', lw1);
    hold on;
    if (any(sen_param(i).time))
    text(sen_param(i).time(1),sen_param(i).r(1)+5,num2str(i), 'FontSize',12)
    end
end
yline(r_min/2, "--k", "LineWidth", lw2);
yline(r_max/2, "--k", "LineWidth", lw2);
xlabel("Time (s)");
ylabel("Distance from sensor (m)");
grid on;

lw = 4;
col = 'k';
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
            %T = inv(T);
            T = eye(4); 
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
for i = 1:num_obstacles
    x = sen_param(i).r .* cosd(sen_param(i).elev) .* cosd(sen_param(i).azim);
    y = sen_param(i).r .* cosd(sen_param(i).elev) .* sind(sen_param(i).azim);
    z = sen_param(i).r .* sind(sen_param(i).elev);
   plot3(x', y', z', "LineWidth", lw1);
   hold on;
end
set(gca,'DataAspectRatio',[1 1 1])
grid on;
title('3D Plot of obstacles in Sensor Frame');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
clear;


