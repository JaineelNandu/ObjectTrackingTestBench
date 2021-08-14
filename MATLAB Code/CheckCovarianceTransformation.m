T = translation([0; 0; 0])*rotation(1, 30); %Assumed Inertial to Sensor
P_inertial = [0.05; 0.07; -0.06; 1];
P_sensor = T*P_inertial;
sensor_std = [0.1; 0.3; 0.5];

probability_manual = (1/(((2*pi)^(3/2))*(sensor_std(1)*sensor_std(2)*sensor_std(3))))*exp((-1/2)*(((P_sensor(1)^2)/(sensor_std(1)^2))+((P_sensor(2)^2)/(sensor_std(2)^2))+((P_sensor(3)^2)/(sensor_std(3)^2))));


cov_S = diag(sensor_std).^2;
R = inv(T);
R = R(1:3, 1:3);

cov_I = R*cov_S*(R');

probability_cov = (1/((2*pi)^(3/2)))*(1/sqrt(det(cov_I)))*exp((-1/2)*(P_inertial(1:3)')*inv(cov_I)*(P_inertial(1:3)));

function rot = rotation(axis, degrees)
    cd = cosd(degrees);
    sd = sind(degrees);
    if (axis == 0)
       rot = [1 0 0 0; 0 cd -sd 0; 0 sd cd 0; 0 0 0 1];
    elseif (axis == 1)
       rot = [cd 0 sd 0; 0 1 0 0; -sd 0 cd 0; 0 0 0 1];
    elseif axis == 2
       rot = [cd -sd 0 0; sd cd 0 0; 0 0 1 0; 0 0 0 1];
    end
end

function tra = translation(vector)
    tra = [1 0 0 vector(1); 0 1 0 vector(2); 
        0 0 1 vector(3); 0 0 0 1];
end