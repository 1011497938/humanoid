clear;
clc;
close all;



A = load('bot5.txt');
% intrinsic = [360.591090231311 360.4918824799427 624.7131585594641 496.0890520277582]; % bot4
% intrinsic = [366.044 366.132 732.081 488.148];
% intrinsic = [359.2305434357884, 359.2036693358589, 631.9867259144648, 460.4559439567722];
% intrinsic = [360.1698179777144, 360.2679184885014, 627.0281224044058, 457.1228031411261];
% intrinsic = [361.007, 361.304, 639.075, 482.132];
intrinsic = [363.6161111787105 363.4026850466549 612.8098230517493 475.433439219378]; % bot5 - new
% intrinsic = [362.669 362.185 592.333 446.13]; % bot2
% intrinsic = [360.1698179777144, 360.2679184885014,627.0281224044058,  457.1228031411261]; % bot 1

fx = intrinsic(1);
fy = intrinsic(2);
cx = intrinsic(3);
cy = intrinsic(4);

camMatrix = [fx 0 cx 0;
                0 fy cy 0;
                0  0  1 0;
                0  0  0 1];

% Xw2p
% Yw2p
% Zw2p

% RXw2p
% RYw2p
% RZw2p

% Xp2c
% Yp2c
% Zp2c

% RXp2c
% RYp2c
% RZp2c

% scaleYaw
% scalePitch
% biasYaw
% biasPitch

%           x       y       z       rx        ry      rz        x        y       z      rx       ry       rz     s    s     b     b
para0  = [  0;      0;     60;       0;       0;       0;       0;       0;      10;      0;       0;       0;    1;   1;    0;    0];    
lb     = [ -20;     -10;     40;    -0.2;    -0.4;    -0.2;      -10;      -10;      0;   -0.1;    -0.1;    -0.1;  0.8; 0.8;  -10 / 180 * pi;    -10 / 180 * pi];
ub     = [  20;      10;     70;     0.2;     0.4;     0.2;       10;       10;      20;   0.1;     0.1;     0.1;  1.3; 1.3;    10 / 180 * pi;    10 / 180 * pi];

% lb     = [ -0;     -0;     52;    -0.0;    -0.1;    -0.0;      -1;      -1;      6;      0;    -0.1;       0;    1;   1;    0;    0];
% ub     = [  0;      0;     54;     0.0;     0.1;     0.0;       1;       1;      8;      0;     0.1;       0;    1;   1;    0;    0];

% TODO(MWX): How to test correctness of projection function

uv = projection(para0, 45 / 180 * pi, 0, 450, 130);

disp(uv);
% uv = projection(para0, 0, 0, 450, -130);

% disp(uv);

options = optimset('Display','iter-detailed','Algorithm','interior-point','FunValCheck','on',...
    'TolFun',10^-6,'LargeScale','off','TolX',10^-6,'MaxFunEvals',10^6,...
   'MaxIter',10000);

[respara, reserror, exitflag, output] = fmincon(@errorfunc, para0, [], [], [], [], lb, ub, [], options);
disp(respara);
% % disp(respara);

%% Plot
hold on;
for i = 1 : size(A, 1)
    plot(A(i, 5), A(i, 6), 'b*')
end


for i = 1 : size(A, 1)
    pitch = A(i, 2) / 180 * pi;
    yaw = A(i, 1) / 180 * pi;
    u = A(i, 3);
    v = A(i, 4);
    [xy] = calc_xy(camMatrix, calc_extrinsic(respara, pitch, yaw), u, v);
    plot(xy(1), xy(2), 'r*');
end
