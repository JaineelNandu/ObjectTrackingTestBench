amax = 3;
vmax = 11;
jmax = 1;
vp = vmax - ((amax^2)/(2*jmax));
vold = 10.9999999;

aold = amax*(((vold-vp)/(vp-vmax))+1);
baserate = 270;
dt = 1/baserate;
t= [dt];
a = [aold];
v = [vold];
for i = 1:100000
    if vold >= vp
        anew = amax*(((vold-vp)/(vp-vmax))+1);
    else
        anew = aold;
    end
    vnew = vold + (aold+anew)*dt/2;
    a = [a anew];
    v = [v vnew];
    t = [t dt];
    aold = anew;
    vold = vnew;
end
size(a)
subplot(3, 1, 1);
plot(v, a, 'r', 'LineWidth', 2)
subplot(3,1,2);
plot(t, a, 'b', 'LineWidth', 2);
subplot(3,1,3);
plot(t,v,'g', 'LineWidth', 2);
a = a(a > 0);
size(a);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
amax = 3;
vmax = 11;
jmax = 1;
vp = vmax - ((amax^2)/(2*jmax));
ti = vp/amax;
vold = 0;

aold = 3;
baserate = 270;
dt = 1/baserate;
t= [0];
a = [aold];
v = [vold];
j_prof = [0];
tsum = 0;
while vold < vmax
    if vold >= vp
        if vold <= vmax
        anew = aold - jmax*dt;%amax*(((vold-vp)/(vp-vmax))+1);
        else
        anew = 0;
        end
    else
        anew = aold;
    end
    jnew = ((anew-aold)/dt);
    vnew = vold + (aold+anew)*dt/2;
    a = [a anew];
    v = [v vnew];
    j_prof = [j_prof jnew];
    tsum = tsum + dt;
    t = [t tsum];
    aold = anew;
    vold = vnew;
end
t_ch = t(a<3);
v_ch = v(a<3);
v_fromexp = -(jmax*(t_ch.^2)/2)+(jmax*ti + amax).*t_ch + (-jmax*(ti^2)/2);
a_fromexp = -(jmax*t_ch) + (jmax*ti + amax);
a_fromV = sqrt(2*jmax*ti*amax + amax^2 -2*jmax*v_ch);


clf
set(gcf,'color','white');

subplot(2,2,1)
plot(t, j_prof, 'm', 'LineWidth', 2)
xline(ti, ":b", "LineWidth", 2);
xline(t(end), ":r", "LineWidth", 2);
title("Jerk vs time");
xlabel("Time (s)");
ylabel("Jerk (m/s^3)");
grid on;
legend("Jerk", "t_i", "t_f", "Location", "best");

subplot(2 , 2, 2);
plot(t(1),a(1), '*b', "MarkerSize", 10)
hold on;
plot(t(end), a(end), '*r', "MarkerSize", 10)
plot(t, a, 'b', 'LineWidth', 2);
plot(t_ch, a_fromexp, '--r', 'LineWidth', 2);
xline(ti, ":m", 'LineWidth', 2);
xline(t(end), ":g", 'LineWidth', 2);
grid on;
title("Acceleration vs time");
xlabel("Time (s)");
ylabel("Magnitude of Acceleration (m/s^2)");
legend("Acceleration at t = 0", "Acceleration at t = t_f", "Max possible Acceleration", "Max Acceleration calculated from velocity [v(t) > v_p]","t_i","t_f", 'location', 'best');
ylim([-0.1 3.1])

subplot(2,2,3);
plot(t(1),v(1), '*g', "MarkerSize", 10);
hold on;
plot(t(end), v(end), '*b', "MarkerSize", 10);
plot(t,v,'g', 'LineWidth', 2);
plot(t_ch, v_fromexp, '--b', 'LineWidth', 2);
xline(ti, ":m", "LineWidth", 2);
xline(t(end), ":r", "LineWidth", 2);
yline(vp, ":c", "LineWidth", 2);
grid on;
title("Velocity vs Time");
xlabel("Time (s)");
ylabel("Magnitude of Velocity (m/s)");
legend("Velocity at t = 0", "Velocity at t = t_f", "Velocity calculated from Verlet Integration", "Velocity calculated with respect to time", "t_i", "t_f", "v_p", "Location" , "best");
ylim([-0.01 11.1])

subplot(2, 2, 4);
plot(v, a, 'r', 'LineWidth', 2)
hold on;
plot(v_ch, a_fromV, '--g', 'LineWidth', 2);
grid on;
ylim([-0.1 3.1]);
title("Acceleration vs Velocity");
xlabel("Magnitude of velocity (m/s)");
ylabel("Magnitude of acceleration (m/s^2)");
xline(vp, ":b", 'LineWidth', 2);
xline(vmax, ":m", 'LineWidth', 2);
legend("Max possible Acceleration", " Max Acceleration calculated from velocity [v(t) > v_p]","v_p","v_m_a_x", 'location', 'southwest');
