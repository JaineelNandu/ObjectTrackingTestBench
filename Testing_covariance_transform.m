T = rotation(1, 20)*rotation(2,-77);

rot_T = T(1:3, 1:3);
std = [0.1 0.3 0.5];
cov_sens = diag(std.^2);

cov_inertial = rot_T*cov_sens*(rot_T');
inv_cov_inertial = inv(cov_inertial);

P = [20.5, 11, 12; 4, 5, 1; 4, 5, 10; 1, 2, 36; 143, 2, 1; -39, -60, 1];
d = [];

for i = 1:6
    d = [d P(i,:)*inv_cov_inertial*P(i,:)'];
end


function rot = rotation(axis, degrees)
    cd = cosd(degrees);
    sd = sind(degrees);
    rot = eye(4);
    if axis == 0
        rot = [1 0 0 0; 0 cd -sd 0; 0 sd cd 0; 0 0 0 1];
    elseif axis == 1
        rot = [cd 0 sd 0; 0 1 0 0; -sd 0 cd 0; 0 0 0 1];
    elseif axis == 2
        rot = [cd -sd 0 0; sd cd 0 0; 0 0 1 0; 0 0 0 1];
    end 
end