function sys_id_plot% plot state estimate
load('sys_id/experiments/steer_ramp_2L.mat');
x = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x(:,i) = stateData(i).X';
end
% normalize to origin
x = bsxfun(@minus,x,[x(1,1);x(2,1);x(3,1);0;0;0]);
u = zeros(2,size(stateData,1));
figure(1);
subplot(1,3,1);
axis([-1,2,-1,2]);
car_plot(x,u);
param = [28,19,0.021,0.57,0.20];
% param = [23.7;17.3;0.021;0.66;0.20];
[sim_x, sim_u] = inc_steer_sim(2,1,0.20,param);
car_plot(sim_x,sim_u,'b');
P = sim_x(1:2,:).';
Q = x(1:2,:).';
cm = DiscreteFrechetDist(P,Q);


load('sys_id/experiments/steer_ramp_3L.mat');
x = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x(:,i) = stateData(i).X';
end
% normalize to origin
x = bsxfun(@minus,x,[x(1,1);x(2,1);x(3,1);0;0;0]);
u = zeros(2,size(stateData,1));
subplot(1,3,2);
axis([-1,2,-1,2]);
car_plot(x,u);
[sim_x, sim_u] = inc_steer_sim(3,1,0.16,param);
car_plot(sim_x,sim_u,'b');
P = sim_x(1:2,:).';
Q = x(1:2,:).';
cm = cm+DiscreteFrechetDist(P,Q);

load('sys_id/experiments/steer_ramp_4L.mat');
x = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x(:,i) = stateData(i).X';
end
% normalize to origin
x = bsxfun(@minus,x,[x(1,1);x(2,1);x(3,1);0;0;0]);
u = zeros(2,size(stateData,1));
subplot(1,3,3);
axis([-1,2,-1,2]);
car_plot(x,u);
[sim_x, sim_u] = inc_steer_sim(4,1,0.16,param);
car_plot(sim_x,sim_u,'b');
P = sim_x(1:2,:).';
Q = x(1:2,:).';
cm = cm+DiscreteFrechetDist(P,Q)
end

