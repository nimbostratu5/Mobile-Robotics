D = dlmread('LandTamerLawn_raw.data');
D(1,:); % gives first row
global dt 
dt = 0.01;
endline = size(D,1);

%data format
%time - speed - curv
%time - nan - nan - x - y - z - roll - pitch - yaw

%initialize variables
current_line = 1;
current_time = D(1,1);
next_time = D(2,1);
mode = 0;

%initial sensor data
xpos = D(current_line,4); ypos = D(current_line,5); yaw = D(current_line,9);
z = [xpos;ypos;yaw];

%parameter initialization
Q = [0.0025, 0, 0; 0, 0.0025, 0; 0, 0, 0.025];
H = [1,0,0; 0,1,0; 0,0,1];
I = eye(3); %identity matrix
xESV = zeros(3, 1);
%initial estimate state vector and covariance for state error
xESV(:,1) = [xpos; ypos; yaw];
pESV = Q; 
counter = 0;
mu_x=[];mu_y=[];mu_theta=[];
z_x=[];z_y=[];z_theta=[];
varr=[]; warr=[];
while current_line < endline
    counter = counter +1;
    current_time = current_time + dt;
    if current_time >= next_time
        current_line = current_line + 1;
        if isnan(D(current_line,2)) %sensor data
            %mode=1 & update sensor variables 
            xpos = D(current_line,4);ypos = D(current_line,5);yaw = D(current_line,9);
            z = [xpos;ypos;yaw];
            mode = 1;
        else %command data
            %mode=0 & update command variables 
            v = D(current_line,2); curv = D(current_line,3);
            u = [v; curv];
            mode = 0;
        end
        next_time = D(current_line,1);
    end  
    if v ~= 0
        [xESV, pESV] = ekf_final(xESV,pESV,u,z,mode);
    end
    %save data for plotting
    mu_x(end+1,:)=[current_time, xESV(1,1)];
    mu_y(end+1,:)=[current_time, z(2,1)];
    mu_theta(end+1,:)=[current_time, z(3,1)];
    z_x(end+1,:)=[current_time, z(1,1)];
    z_y(end+1,:)=[current_time, z(2,1)];
    z_theta(end+1,:)=[current_time, z(3,1)];
    varr(end+1,:)=[current_time, v];
    warr(end+1,:)=[current_time, (v*curv)];
    mode = 0;
    
end
%% Plot the results
% Plot the states and measurement
figure(1);
subplot(2,1,1);
plot(mu_x(:,1),mu_x(:,2));
xlabel('t (s)'); ylabel('mu_x (m)'); grid on;
title('mu_x vs. t');
subplot(2,1,2);
plot(z_x(:,1),z_x(:,2));
xlabel('t (s)'); ylabel('z_x (m)'); grid on;
title('z_x vs. t');

figure(2);
subplot(2,1,1);
plot(mu_y(:,1),mu_y(:,2));
xlabel('t (s)'); ylabel('mu_y (m)'); grid on;
title('mu_y vs. t');
subplot(2,1,2);
plot(z_y(:,1),z_y(:,2));
xlabel('t (s)'); ylabel('z_y (m)'); grid on;
title('z_y vs. t');

figure(3);
subplot(2,1,1);
plot(mu_theta(:,1),mu_theta(:,2));
xlabel('t (s)'); ylabel('mu_{theta} (rad)'); grid on;
title('mu_{theta} vs. t');
subplot(2,1,2);
plot(z_theta(:,1),z_theta(:,2));
xlabel('t (s)'); ylabel('z_{theta} (rad)'); grid on;
title('z_{theta} vs. t');

figure(4);
subplot(2,1,1);
plot(mu_y(:,2),mu_x(:,2));
xlim([-22 -19]);
ylim([6 9]);
xlabel('mu_y (m)'); ylabel('mu_x (m)'); grid on;
title('mu_x vs. mu_y');
subplot(2,1,2);
plot(z_y(:,2),z_x(:,2));
xlim([-22 -19]);
ylim([6 9]);
xlabel('z_y (m)'); ylabel('z_x (m)'); grid on;
title('z_x vs. z_y');

figure(5);
subplot(2,1,1);
plot(varr(:,1),varr(:,2));
xlabel('t (s)'); ylabel('vt (m/s)'); grid on;
title('vt vs. t');
subplot(2,1,2);
plot(warr(:,1),warr(:,2));
xlabel('t (s)'); ylabel('w (rad/s)'); grid on;
title('w vs. t');


%%BELOW USED FOR SPECIFIC QUESTIONS/PURPOSES - uncomment if needed

% figure(6);
% subplot(2,1,1);
% plot(mu_theta(:,1),mu_theta(:,2));
% xlim([30 60]);
% xlabel('t (s)'); ylabel('mu_{theta} (rad)'); grid on;
% title('mu_{theta} vs. t');
% subplot(2,1,2);
% plot(z_theta(:,1),z_theta(:,2));
% xlim([30 60]);
% xlabel('t (s)'); ylabel('z_{theta} (rad)'); grid on;
% title('z_{theta} vs. t');

% figure(7);
% subplot(2,1,1);
% plot(varr(:,1),varr(:,2));
% xlim([268 269]);
% xlabel('t (s)'); ylabel('vt (m/s)'); grid on;
% title('vt vs. t');
% subplot(2,1,2);
% plot(warr(:,1),warr(:,2));
% xlim([268 269]);
% xlabel('t (s)'); ylabel('w (rad/s)'); grid on;
% title('w vs. t');


